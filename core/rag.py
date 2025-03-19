import os
from langchain_community.document_loaders import PyPDFDirectoryLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_huggingface import HuggingFaceEmbeddings
from langchain_community.vectorstores import Chroma

class RAG:
    def __init__(self, data_path: str = '/app/my-persona'):
        print('--- RAG INIT ---', flush=True)
        print('Loading data..', flush=True)
         
        pdf_loader = PyPDFDirectoryLoader(data_path)
        pdf_data = pdf_loader.load()

        print('Splitting text', flush=True)
        text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=1000, 
            chunk_overlap=200
        )
        splits = text_splitter.split_documents(pdf_data)

        print('Embedding text', flush=True)
        embeddings = HuggingFaceEmbeddings(
            model_kwargs={'device': 'cpu'}
        )
        
        print('Creating vector store', flush=True)
        self.vectordb = Chroma.from_documents(
            documents=splits, 
            embedding=embeddings
        )
        
        print('Creating retriever', flush=True)
        self.retriever = self.vectordb.as_retriever()

        print('--- RAG INIT COMPLETE ---', flush=True)
    
    def format_docs(self, docs):
        return "\n\n".join(doc.page_content for doc in docs)