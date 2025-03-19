import os
from openai import OpenAI
from settings import OPENAI_API_KEY
from settings import PROMPT

class CustomGPT:
    def __init__(self) -> None:
        self.client = OpenAI(api_key=OPENAI_API_KEY)
        self.system_prompt = PROMPT
        
        self.messages = [
            {
                "role": "system", 
                "content": self.system_prompt
            } 
        ]
    
    def exec(self, query: str, role: str = "user") -> str:
        reply = ''
        if query:
            self.messages.append( 
                {"role": role, "content": query}, 
            ) 
            chat = self.client.chat.completions.create(
                model="gpt-4o", 
                temperature=0.5,
                messages=self.messages) 
            
            reply = chat.choices[0].message.content 
            self.messages.append({"role": "assistant", "content": reply})
        return reply

    def exec_gradio(self, history: list, query: str, role: str = "user") -> str:
        if isinstance(query, tuple):
            reply = "Sorry, this is not implemented yet."
        else:
            self.messages.append( 
                {"role": role, "content": query}, 
            ) 
            chat = self.client.chat.completions.create(
                model="gpt-4-turbo", 
                temperature=0.5,
                messages=self.messages
            ) 
            
            reply = chat.choices[0].message.content 
            self.messages.append({"role": "assistant", "content": reply})
        
        return reply