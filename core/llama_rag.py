from langchain_ollama import ChatOllama
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.messages import HumanMessage
from langchain.chains.combine_documents import create_stuff_documents_chain
from langchain.chains import create_history_aware_retriever, create_retrieval_chain
from rag import RAG
from settings import PROMPT

class OllamaRag:
    def __init__(self):
        self.llm = ChatOllama(
            model="llama3.2:3b",
            temperature=0.01,

        )
        self.rag = RAG()

        self.chat_history = list()
        contextualize_q_system_prompt = self.prompt_template = """
        <|begin_of_text|><|start_header_id|>system<|end_header_id|>

        Cutting Knowledge Date: December 2023
        Today Date: 23 July 2024

        Given a chat history and the latest user question \
        which might reference context in the chat history, formulate a standalone question \
        which can be understood without the chat history. Do NOT answer the question, \
        just reformulate it if needed and otherwise return it as is.<|eot_id|>
        <|start_header_id|>user<|end_header_id|>
        Chat History: {chat_history}
        ---
        Context: {context}
        ---
        Question: {input}<|eot_id|>
        <|start_header_id|>assistant<|end_header_id|>
        """

        contextualize_q_prompt = ChatPromptTemplate.from_template(
            contextualize_q_system_prompt
        )

        self.history_aware_retriever = create_history_aware_retriever(
            self.llm, 
            self.rag.retriever, 
            contextualize_q_prompt
        )

        self.prompt_template = """
        <|begin_of_text|><|start_header_id|>system<|end_header_id|>

        Cutting Knowledge Date: December 2023
        Today Date: 15 March 2025

        {prompt}<|eot_id|>
        <|start_header_id|>user<|end_header_id|>
        Chat History: {chat_history}
        ---
        Context: {context}
        ---
        Question: {input}<|eot_id|>
        <|start_header_id|>assistant<|end_header_id|>
        """

        prompt = ChatPromptTemplate.from_template(
            self.prompt_template
        )

        question_answer_chain = create_stuff_documents_chain(
            self.llm, prompt
        )

        self.chain = create_retrieval_chain(
            self.history_aware_retriever, question_answer_chain
        ) 
    
    def exec(self, query: str, role: str = "user"):
        response = self.chain.invoke(
            {   
                'prompt': PROMPT,
                'chat_history': self.chat_history,
                'input': query,
                'context': self.rag.retriever | self.rag.format_docs
            }
        )
        self.chat_history.extend([
            HumanMessage(content=query), response['answer']
        ])

        return response['answer']