import os
import numpy as np
from numpy import dot, sqrt
import cv2
import onnxruntime
import torch
import torchvision.transforms as transforms

from align_faces import warp_and_crop_face, get_reference_facial_points
from retinaface.detector import RetinafaceDetector

class Arc:
    def __init__(self):

        self.tface_path = 'models/arcface/tface101.onnx'

        # Initialise Face Detector
        self.face_detector = None
        # Initialise Face Embedding Extractor
        self.session = onnxruntime.InferenceSession(self.tface_path, providers=["CPUExecutionProvider"])

    def get_test_transform(self):
        test_transform = transforms.Compose([    
                transforms.ToTensor(),    
                transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])])
        return test_transform

    # Cropping face 
    def preprocess_face(self, img, target_size=(112,112)):
        img = cv2.imread(img)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (112, 112))
        img = self.get_test_transform()(img)
        img = img.unsqueeze_(0)
        return img

    # Tface embeddings 
    def l2_norm(self, x):
        """ 
            l2 normalize
        """
        output = x / np.linalg.norm(x)

        return output

    def to_numpy(self, tensor):
        return tensor.detach().cpu().numpy() if tensor.requires_grad else tensor.cpu().numpy()

    def get_embeddings(self, path):
        img = self.preprocess_face(path)
        inputs = {self.session.get_inputs()[0].name: self.to_numpy(img)}    
        outs = self.session.run(None, inputs)[0].squeeze()
        return outs
    
    def get_runtime_embeddings(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (112, 112))
        img = self.get_test_transform()(img)
        img = img.unsqueeze_(0)
        inputs = {self.session.get_inputs()[0].name: self.to_numpy(img)}    
        outs = self.session.run(None, inputs)[0].squeeze()
        return outs

class Aligner:
    def __init__(self):
        # Load face detection model
        self.retinaface_model = RetinafaceDetector(type='cpu')
        self.desired_size = 512

        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    # align faces 
    def process(self, img, output_size=(512,512)):
        _, facial5points = self.retinaface_model.detect_faces(img)
        dst_img = None
        ret = False
        try:
            facial5points = np.reshape(facial5points[0], (2, 5))
            ret = True
            default_square = True
            inner_padding_factor = 0.7
            outer_padding = (0, 0)

            # get the reference 5 landmarks position in the crop settings
            reference_5pts = get_reference_facial_points(
                output_size, inner_padding_factor, outer_padding, default_square)

            dst_img = warp_and_crop_face(img, facial5points, reference_pts=reference_5pts, crop_size=output_size)
            return ret, dst_img 

        except:
            return ret, img
    
    def pad(self, frame):
        old_size = frame.shape[:2] # old_size is in (height, width) format
        ratio = float(self.desired_size)/max(old_size)
        new_size = tuple([int(x*ratio) for x in old_size])
        # new_size should be in (width, height) format
        im = cv2.resize(frame, (new_size[1], new_size[0]))
        delta_w = self.desired_size - new_size[1]
        delta_h = self.desired_size - new_size[0]
        top, bottom = delta_h//2, delta_h-(delta_h//2)
        left, right = delta_w//2, delta_w-(delta_w//2)
        color = [0, 0, 0]
        padded_frame = cv2.copyMakeBorder(
            im, top, bottom, left, right, cv2.BORDER_CONSTANT,
            value=color)
        return padded_frame

    def align(self, dir, img):
        frame = cv2.imread(os.path.join(dir, img))
        frame = self.pad(frame)
        ret, aligned_face = self.process(frame, output_size=(512, 512))
        return aligned_face
    
    def align_runtime(self, frame):
        frame = self.pad(frame)
        ret, aligned_face = self.process(frame, output_size=(512, 512))
        return aligned_face

class FaceRec:
    def cosine_similarity(self, x, y):
        return dot(x, y) / (sqrt(dot(x, x)) * sqrt(dot(y, y)))

    def cosine_matcher(self, embds):
        high = -999
        name = None

        for tname, dbembd in self.embeddings_db.items():
            sim = self.cosine_similarity(dbembd, embds)
            if sim > high:
                high = sim
                name = tname
     
        return name, high
    
    def create_aligned_faces(
            self, 
            facedirpath:str, 
            aligndirpath:str
        ) -> None:
        facedir = os.listdir(facedirpath)
        os.makedirs(aligndirpath, exist_ok=True)

        for img in facedir:
            try:
                aligned_face = self.aligner.align(facedirpath, img)
                cv2.imwrite(os.path.join(aligndirpath, img), aligned_face)
            except:
                print('Failed to align face', flush=True)
        print('Faces aligned successfully!', flush=True)

    def create_embeddings_db(self, aligndirpath: str) -> None:
        aligned_faces = os.listdir(aligndirpath)
        for face in aligned_faces:
            name = face.split('.')[0]
            embd = self.arc.get_embeddings(os.path.join(aligndirpath, face))
            self.embeddings_db.update({name.upper(): embd})
        print('Embeddings DB created successfully!', flush=True)
    
    def register_face(self, name, img):
        aligned_face = self.aligner.align_runtime(img)
        embd = self.arc.get_runtime_embeddings(aligned_face)
        self.embeddings_db.update({name.upper(): embd})
        print('{} registered successfully!'.format(name), flush=True)

    def delete_face(self, name):
        self.embeddings_db.pop(name)
        print('{} deleted successfully!'.format(name.upper()), flush=True)
        
    def match_face(self, img):
        aligned_face = self.aligner.align_runtime(img)
        embd = self.arc.get_runtime_embeddings(aligned_face)
        name, sim = self.cosine_matcher(embd)
        return name, sim

    def __init__(
            self,
            facedirpath: str = 'faces/',
            aligndirpath: str = 'faces/aligned_faces/'
        ):

        self.arc = Arc()
        self.aligner = Aligner()
        self.embeddings_db = {}

        # Align faces
        self.create_aligned_faces(facedirpath, aligndirpath)

        # Create embeddings
        self.create_embeddings_db(aligndirpath)

