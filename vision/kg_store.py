"""
Knowledge graph storage — thin Kuzu wrapper.

Schema:
  Node tables  — Object, Event, Person, Image, Diary
  Rel tables   — INVOLVES (Event→Object), WITNESSED_BY (Event→Person),
                 NEAR (Object→Object), PICTURED_IN (Object→Image),
                 CAPTURED_AT (Event→Image), APPEARS_IN (Person→Image)

Concurrency model:
  - KGStore(read_only=False): used exclusively by kg_builder.py (single process,
    single thread).  Protected by an internal threading.Lock.
  - KGStore(read_only=True):  used by bridge-api.py for read endpoints.
    Multiple read-only DB handles can coexist with a single write handle.
"""
from __future__ import annotations

import base64
import json
import os
import threading
import time
import uuid
from pathlib import Path
from typing import Optional

_HERE = Path(__file__).parent
_ROOT = _HERE.parent

_DB_DIR     = os.getenv("KG_DB_DIR",     str(_ROOT / "data" / "kg" / "graph_db"))
_IMAGES_DIR = os.getenv("KG_IMAGES_DIR", str(_ROOT / "data" / "kg" / "images"))

_SCHEMA = [
    # Node tables
    "CREATE NODE TABLE IF NOT EXISTS Object(id STRING, label STRING, first_seen DOUBLE, last_seen DOUBLE, attrs STRING, PRIMARY KEY(id))",
    "CREATE NODE TABLE IF NOT EXISTS Event(id STRING, content STRING, ts DOUBLE, PRIMARY KEY(id))",
    "CREATE NODE TABLE IF NOT EXISTS Person(id STRING, name STRING, first_seen DOUBLE, last_seen DOUBLE, PRIMARY KEY(id))",
    "CREATE NODE TABLE IF NOT EXISTS Image(id STRING, ts DOUBLE, path STRING, PRIMARY KEY(id))",
    "CREATE NODE TABLE IF NOT EXISTS Diary(id STRING, date STRING, text STRING, generated_at DOUBLE, PRIMARY KEY(id))",
    # Relationship tables
    "CREATE REL TABLE IF NOT EXISTS INVOLVES(FROM Event TO Object, ts DOUBLE)",
    "CREATE REL TABLE IF NOT EXISTS WITNESSED_BY(FROM Event TO Person, ts DOUBLE)",
    "CREATE REL TABLE IF NOT EXISTS NEAR(FROM Object TO Object, ts DOUBLE)",
    "CREATE REL TABLE IF NOT EXISTS PICTURED_IN(FROM Object TO Image)",
    "CREATE REL TABLE IF NOT EXISTS CAPTURED_AT(FROM Event TO Image)",
    "CREATE REL TABLE IF NOT EXISTS APPEARS_IN(FROM Person TO Image)",
]


class KGStore:
    def __init__(
        self,
        db_dir: str = _DB_DIR,
        images_dir: str = _IMAGES_DIR,
        read_only: bool = False,
    ):
        import kuzu  # imported here so the rest of the codebase doesn't hard-require kuzu

        self._images = Path(images_dir)
        self._read_only = read_only
        self._lock = threading.Lock()

        # Kuzu creates db_dir itself — only ensure the parent exists.
        Path(db_dir).parent.mkdir(parents=True, exist_ok=True)
        self._images.mkdir(parents=True, exist_ok=True)

        self._db = kuzu.Database(db_dir, read_only=read_only)
        self._conn = kuzu.Connection(self._db)

        if not read_only:
            self._bootstrap()

    # ── Schema ────────────────────────────────────────────────────────────────

    def _bootstrap(self):
        with self._lock:
            for stmt in _SCHEMA:
                try:
                    self._conn.execute(stmt)
                except Exception as e:
                    # Table already exists is fine; anything else re-raise
                    if "already exists" not in str(e).lower():
                        raise

    # ── Image helpers ─────────────────────────────────────────────────────────

    def save_image(self, b64_jpeg: str) -> tuple[str, str]:
        """Decode and persist a base64 JPEG. Returns (image_id, absolute_path)."""
        img_id = str(uuid.uuid4())
        path = self._images / f"{img_id}.jpg"
        with open(path, "wb") as f:
            f.write(base64.b64decode(b64_jpeg))
        return img_id, str(path)

    def image_path(self, img_id: str) -> Optional[Path]:
        p = self._images / f"{img_id}.jpg"
        return p if p.exists() else None

    def _add_image_node(self, img_id: str, path: str, ts: float):
        try:
            self._conn.execute(
                "CREATE (:Image {id: $id, ts: $ts, path: $path})",
                {"id": img_id, "ts": ts, "path": path},
            )
        except Exception:
            pass  # duplicate key on second node sharing the same image is expected

    # ── Write API ─────────────────────────────────────────────────────────────

    def add_object(self, label: str, attrs: dict, img_id: str, img_path: str, ts: float) -> str:
        """Upsert an Object node and link to the given Image. Returns node id."""
        with self._lock:
            # Check existence
            res = self._conn.execute(
                "MATCH (o:Object {label: $label}) RETURN o.id", {"label": label}
            )
            if res.has_next():
                node_id = res.get_next()[0]
                self._conn.execute(
                    "MATCH (o:Object {id: $id}) SET o.last_seen = $ts",
                    {"id": node_id, "ts": ts},
                )
            else:
                node_id = str(uuid.uuid4())
                self._conn.execute(
                    "CREATE (:Object {id: $id, label: $label, first_seen: $ts, last_seen: $ts, attrs: $attrs})",
                    {"id": node_id, "label": label, "ts": ts, "attrs": json.dumps(attrs)},
                )

            if img_id:
                self._add_image_node(img_id, img_path, ts)
                self._conn.execute(
                    "MATCH (o:Object {id: $oid}), (i:Image {id: $iid}) CREATE (o)-[:PICTURED_IN]->(i)",
                    {"oid": node_id, "iid": img_id},
                )
            return node_id

    def add_event(self, description: str, involves_labels: list[str], img_id: str, img_path: str, ts: float) -> str:
        """Create an Event node, link to involved Objects, and link to Image."""
        with self._lock:
            node_id = str(uuid.uuid4())
            self._conn.execute(
                "CREATE (:Event {id: $id, content: $content, ts: $ts})",
                {"id": node_id, "content": description, "ts": ts},
            )
            for label in involves_labels:
                res = self._conn.execute(
                    "MATCH (o:Object {label: $label}) RETURN o.id", {"label": label}
                )
                if res.has_next():
                    obj_id = res.get_next()[0]
                    self._conn.execute(
                        "MATCH (e:Event {id: $eid}), (o:Object {id: $oid}) CREATE (e)-[:INVOLVES {ts: $ts}]->(o)",
                        {"eid": node_id, "oid": obj_id, "ts": ts},
                    )
            if img_id:
                self._add_image_node(img_id, img_path, ts)
                self._conn.execute(
                    "MATCH (e:Event {id: $eid}), (i:Image {id: $iid}) CREATE (e)-[:CAPTURED_AT]->(i)",
                    {"eid": node_id, "iid": img_id},
                )
            return node_id

    def add_person(self, name: str, img_id: str, img_path: str, ts: float) -> str:
        """Upsert a Person node and link to Image. Returns node id."""
        with self._lock:
            res = self._conn.execute(
                "MATCH (p:Person {name: $name}) RETURN p.id", {"name": name}
            )
            if res.has_next():
                node_id = res.get_next()[0]
                self._conn.execute(
                    "MATCH (p:Person {id: $id}) SET p.last_seen = $ts",
                    {"id": node_id, "ts": ts},
                )
            else:
                node_id = str(uuid.uuid4())
                self._conn.execute(
                    "CREATE (:Person {id: $id, name: $name, first_seen: $ts, last_seen: $ts})",
                    {"id": node_id, "name": name, "ts": ts},
                )
            if img_id:
                self._add_image_node(img_id, img_path, ts)
                self._conn.execute(
                    "MATCH (p:Person {id: $pid}), (i:Image {id: $iid}) CREATE (p)-[:APPEARS_IN]->(i)",
                    {"pid": node_id, "iid": img_id},
                )
            return node_id

    def save_diary(self, date: str, text: str):
        with self._lock:
            # Delete existing entry for this date first
            try:
                self._conn.execute(
                    "MATCH (d:Diary {date: $date}) DETACH DELETE d", {"date": date}
                )
            except Exception:
                pass
            self._conn.execute(
                "CREATE (:Diary {id: $id, date: $date, text: $text, generated_at: $ts})",
                {"id": str(uuid.uuid4()), "date": date, "text": text, "ts": time.time()},
            )

    # ── Read API ──────────────────────────────────────────────────────────────

    def existing_labels(self) -> set[str]:
        """Return lowercase set of all known object labels, event descriptions, person names."""
        labels: set[str] = set()
        for query, field in [
            ("MATCH (o:Object) RETURN o.label", None),
            ("MATCH (e:Event) RETURN e.content", None),
            ("MATCH (p:Person) RETURN p.name", None),
        ]:
            res = self._conn.execute(query)
            while res.has_next():
                val = res.get_next()[0]
                if val:
                    labels.add(val.lower())
        return labels

    def query_by_label(self, label: str) -> list[dict]:
        """Find nodes whose name/label/description contains `label` (case-insensitive)."""
        label_lc = label.lower()
        hits: list[dict] = []

        res = self._conn.execute(
            "MATCH (o:Object) WHERE lower(o.label) CONTAINS $q "
            "OPTIONAL MATCH (o)-[:PICTURED_IN]->(i:Image) "
            "RETURN o.id, o.label, o.last_seen, i.id ORDER BY o.last_seen DESC LIMIT 5",
            {"q": label_lc},
        )
        while res.has_next():
            row = res.get_next()
            hits.append({"type": "object", "id": row[0], "label": row[1], "last_seen": row[2], "image_id": row[3]})

        res = self._conn.execute(
            "MATCH (e:Event) WHERE lower(e.content) CONTAINS $q "
            "OPTIONAL MATCH (e)-[:CAPTURED_AT]->(i:Image) "
            "RETURN e.id, e.content, e.ts, i.id ORDER BY e.ts DESC LIMIT 5",
            {"q": label_lc},
        )
        while res.has_next():
            row = res.get_next()
            hits.append({"type": "event", "id": row[0], "label": row[1], "last_seen": row[2], "image_id": row[3]})

        res = self._conn.execute(
            "MATCH (p:Person) WHERE lower(p.name) CONTAINS $q "
            "OPTIONAL MATCH (p)-[:APPEARS_IN]->(i:Image) "
            "RETURN p.id, p.name, p.last_seen, i.id ORDER BY p.last_seen DESC LIMIT 5",
            {"q": label_lc},
        )
        while res.has_next():
            row = res.get_next()
            hits.append({"type": "person", "id": row[0], "label": row[1], "last_seen": row[2], "image_id": row[3]})

        return sorted(hits, key=lambda x: x["last_seen"] or 0, reverse=True)

    def nodes_since(self, since_ts: float) -> list[dict]:
        """Return all nodes first seen at or after since_ts."""
        nodes: list[dict] = []

        res = self._conn.execute(
            "MATCH (o:Object) WHERE o.first_seen >= $ts RETURN o.id, o.label, o.first_seen, o.last_seen, o.attrs",
            {"ts": since_ts},
        )
        while res.has_next():
            row = res.get_next()
            nodes.append({"type": "object", "id": row[0], "label": row[1], "first_seen": row[2], "last_seen": row[3], "attrs": row[4]})

        res = self._conn.execute(
            "MATCH (e:Event) WHERE e.ts >= $ts RETURN e.id, e.content, e.ts",
            {"ts": since_ts},
        )
        while res.has_next():
            row = res.get_next()
            nodes.append({"type": "event", "id": row[0], "label": row[1], "first_seen": row[2], "last_seen": row[2]})

        res = self._conn.execute(
            "MATCH (p:Person) WHERE p.first_seen >= $ts RETURN p.id, p.name, p.first_seen, p.last_seen",
            {"ts": since_ts},
        )
        while res.has_next():
            row = res.get_next()
            nodes.append({"type": "person", "id": row[0], "label": row[1], "first_seen": row[2], "last_seen": row[3]})

        return nodes

    def graph_snapshot(self) -> dict:
        """Full node+edge snapshot for vis-network dashboard rendering."""
        nodes: list[dict] = []
        edges: list[dict] = []

        # Object nodes
        res = self._conn.execute("MATCH (o:Object) RETURN o.id, o.label, o.first_seen, o.last_seen")
        while res.has_next():
            row = res.get_next()
            nodes.append({"id": row[0], "label": row[1], "type": "object", "first_seen": row[2], "last_seen": row[3]})

        # Event nodes
        res = self._conn.execute("MATCH (e:Event) RETURN e.id, e.content, e.ts")
        while res.has_next():
            row = res.get_next()
            nodes.append({"id": row[0], "label": row[1], "type": "event", "first_seen": row[2], "last_seen": row[2]})

        # Person nodes
        res = self._conn.execute("MATCH (p:Person) RETURN p.id, p.name, p.first_seen, p.last_seen")
        while res.has_next():
            row = res.get_next()
            nodes.append({"id": row[0], "label": row[1], "type": "person", "first_seen": row[2], "last_seen": row[3]})

        # INVOLVES edges
        res = self._conn.execute("MATCH (e:Event)-[:INVOLVES]->(o:Object) RETURN e.id, o.id")
        while res.has_next():
            row = res.get_next()
            edges.append({"from": row[0], "to": row[1], "rel": "involves"})

        # WITNESSED_BY edges
        res = self._conn.execute("MATCH (e:Event)-[:WITNESSED_BY]->(p:Person) RETURN e.id, p.id")
        while res.has_next():
            row = res.get_next()
            edges.append({"from": row[0], "to": row[1], "rel": "witnessed_by"})

        # NEAR edges
        res = self._conn.execute("MATCH (a:Object)-[:NEAR]->(b:Object) RETURN a.id, b.id")
        while res.has_next():
            row = res.get_next()
            edges.append({"from": row[0], "to": row[1], "rel": "near"})

        return {"nodes": nodes, "edges": edges}

    def get_node_images(self, node_id: str) -> list[dict]:
        """All images attached to a given node id (checked across all node types)."""
        images: list[dict] = []
        for rel, node_t in [("PICTURED_IN", "Object"), ("CAPTURED_AT", "Event"), ("APPEARS_IN", "Person")]:
            try:
                res = self._conn.execute(
                    f"MATCH (n:{node_t} {{id: $id}})-[:{rel}]->(i:Image) "
                    "RETURN i.id, i.ts, i.path ORDER BY i.ts DESC",
                    {"id": node_id},
                )
                while res.has_next():
                    row = res.get_next()
                    images.append({"id": row[0], "ts": row[1], "path": row[2]})
            except Exception:
                pass
        return sorted(images, key=lambda x: x["ts"], reverse=True)

    def get_node_detail(self, node_id: str) -> Optional[dict]:
        """Full node info + connected node ids + images."""
        detail = None
        for node_t, label_field, ts_field in [
            ("Object", "label", "first_seen"),
            ("Event", "content", "ts"),
            ("Person", "name", "first_seen"),
        ]:
            try:
                res = self._conn.execute(
                    f"MATCH (n:{node_t} {{id: $id}}) RETURN n.id, n.{label_field}, n.{ts_field}",
                    {"id": node_id},
                )
                if res.has_next():
                    row = res.get_next()
                    detail = {"id": row[0], "label": row[1], "type": node_t.lower(), "ts": row[2]}
                    break
            except Exception:
                pass
        if detail is None:
            return None
        detail["images"] = self.get_node_images(node_id)
        return detail

    def load_diary(self, date: str) -> Optional[str]:
        res = self._conn.execute(
            "MATCH (d:Diary {date: $date}) RETURN d.text", {"date": date}
        )
        if res.has_next():
            return res.get_next()[0]
        return None

    def count_nodes(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for nt in ("Object", "Event", "Person", "Image"):
            res = self._conn.execute(f"MATCH (n:{nt}) RETURN count(n)")
            counts[nt.lower()] = res.get_next()[0] if res.has_next() else 0
        return counts

    def touch_label(self, label: str, ts: float):
        """Update last_seen for an Object with the given label (no-op if not found)."""
        with self._lock:
            try:
                self._conn.execute(
                    "MATCH (o:Object {label: $label}) SET o.last_seen = $ts",
                    {"label": label, "ts": ts},
                )
            except Exception:
                pass

    def reset(self):
        """Wipe all data. Caller is responsible for deleting image files separately."""
        with self._lock:
            for nt in ("Object", "Event", "Person", "Image", "Diary"):
                try:
                    self._conn.execute(f"MATCH (n:{nt}) DETACH DELETE n")
                except Exception:
                    pass


# ── Self-test (python vision/kg_store.py --self-test) ────────────────────────

if __name__ == "__main__":
    import os, shutil, sys, tempfile

    if "--self-test" not in sys.argv:
        print("Usage: python vision/kg_store.py --self-test")
        sys.exit(0)

    tmp_root = tempfile.mkdtemp(suffix="_kg_test")
    tmp_db   = os.path.join(tmp_root, "graph_db")    # Kuzu will create this
    tmp_img  = os.path.join(tmp_root, "images")
    try:
        store = KGStore(db_dir=tmp_db, images_dir=tmp_img)
        now = time.time()

        # Insert Object
        img_id, img_path = str(uuid.uuid4()), str(Path(tmp_img) / "test.jpg")
        # Write a dummy JPEG header so save_image has something to decode
        open(img_path, "wb").write(b"dummy")
        obj_id = store.add_object("mug", {"color": "blue"}, img_id, img_path, now)
        print(f"  Object: {obj_id[:8]}…  label=mug")

        # Insert Event
        evt_id = store.add_event("mug on desk", ["mug"], img_id, img_path, now)
        print(f"  Event:  {evt_id[:8]}…  desc='mug on desk'")

        # Insert Person
        per_id = store.add_person("akash", img_id, img_path, now)
        print(f"  Person: {per_id[:8]}…  name=akash")

        # Read back
        hits = store.query_by_label("mug")
        assert any(h["label"] == "mug" for h in hits), "mug not found"
        print(f"  query_by_label('mug') → {len(hits)} hits  ✓")

        labels = store.existing_labels()
        assert "mug" in labels and "akash" in labels, "labels missing"
        print(f"  existing_labels() → {sorted(labels)[:5]}  ✓")

        snap = store.graph_snapshot()
        assert len(snap["nodes"]) == 3, f"expected 3 nodes, got {len(snap['nodes'])}"
        print(f"  graph_snapshot() → {len(snap['nodes'])} nodes, {len(snap['edges'])} edges  ✓")

        # Diary
        store.save_diary("2026-05-08", "Saw a mug today.")
        text = store.load_diary("2026-05-08")
        assert text == "Saw a mug today.", f"diary mismatch: {text!r}"
        print(f"  diary round-trip  ✓")

        counts = store.count_nodes()
        print(f"  counts: {counts}")

        store.reset()
        counts = store.count_nodes()
        assert all(v == 0 for v in counts.values()), f"reset failed: {counts}"
        print(f"  reset()  ✓")

        print("\nAll self-tests passed.")
    finally:
        shutil.rmtree(tmp_root, ignore_errors=True)
