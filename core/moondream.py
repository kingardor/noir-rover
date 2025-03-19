from transformers import AutoModelForCausalLM, AutoTokenizer
from PIL import Image

class Moondream:
    def __init__(self) -> None:
        print('Initializing Moondream...')
        self.model = AutoModelForCausalLM.from_pretrained(
            "vikhyatk/moondream2",
            revision="2025-01-09",
            trust_remote_code=True,
            device_map={"": "cuda"}
        )
        print('Moondream initialized.')

    def short_caption(self, image: Image) -> str:
        res = self.model.caption(image, length="short")["caption"]
        return res

    def caption(self, image: Image) -> str:
        res = ""
        for t in self.model.caption(image, length="normal", stream=True)["caption"]:
            res += t
        return res

    def query(self, image: Image, question: str) -> dict:
        res = self.model.query(image, question)["answer"]
        return res

    def detect(self, image: Image, object: str) -> dict:
        res = self.model.detect(image, object)
        return res
    
    def point(self, image: Image, object: str) -> dict:
        res = self.model.point(image, object)
        return res