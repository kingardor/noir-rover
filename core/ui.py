import customtkinter
from customtkinter import END
import redis

class UserInterface:

    def start_ui(self) -> None:
        self.app.mainloop()

    def speak(self) -> None:
        if self.chat_entry.get():
            self.cache.set('prompt', '[TEXT]' + self.chat_entry.get())
            self.cache.set('prompt-lang', 'en')
            self.chat_entry.delete(0, END)

    def clear(self) -> None:
        self.my_text.delete(1.0, END)
        self.chat_entry.delete(0, END)

    def add_text(self, type: str, text: str) -> None:
        self.my_text.insert(END, f"\n\n{type}: {text}")
    
    def update_frame(self):
        if self.frame is not None:
            try:
                w, h = self.frame.size
                photo = customtkinter.CTkImage(
                    self.frame,
                    size=(int(w * 0.75), int(h * 0.75))
                )
                self.camera.configure(image=photo)
                self.camera.image = photo
                self.frame = None
            except:
                pass
        self.camera.after(250, self.update_frame)

    def construct_ui(self) -> None:
        # Create a main frame to hold both the text and camera
        main_frame = customtkinter.CTkFrame(
            master=self.app,
            fg_color="#717378"
        )
        main_frame.pack(pady=5, padx=5, fill="both", expand=True)

        # Create Text Frame (Left side)
        text_frame = customtkinter.CTkFrame(
            master=main_frame,
            fg_color="#717378"
        )
        text_frame.pack(side="left", padx=5, pady=5, fill="both", expand=True)

        # Add Text Widget to view conversation
        self.my_text = customtkinter.CTkTextbox(
            master=text_frame,
            width=400,
            height=400,
            border_width=2,
            font=("NotoSansDevanagari_Condensed-Regular", 14)  # Set the font to support Devanagari
        )
        self.my_text.pack(side="left", fill="both", expand=True)

        # Create Scrollbar for text widget
        text_scroll = customtkinter.CTkScrollbar(
            master=text_frame,
            command=self.my_text.yview
        )
        text_scroll.pack(side="right", fill="y")

        # Add the scrollbar to the textbox
        self.my_text.configure(yscrollcommand=text_scroll.set)

        # Camera Frame (Right side)
        self.camera = customtkinter.CTkLabel(
            master=main_frame,
            width=960,
            height=540,
            text=""
        )
        self.camera.pack(side="right", padx=5, pady=5, fill="both", expand=True)

        # Entry Widget To type stuff to chat
        self.chat_entry = customtkinter.CTkEntry(
            self.app,
            placeholder_text="Type something, human..",
            width=400,
            height=30,
            border_width=2,
            font=("Arial", 12)  # Set the font to support Devanagari
        )
        self.chat_entry.pack(pady=10)

        # Create Button Frame
        button_frame = customtkinter.CTkFrame(
            master=self.app,
            fg_color="#242424"
        )
        button_frame.pack(pady=10)

        # Create Submit Button
        submit_button = customtkinter.CTkButton(
            master=button_frame,
            text="Send",
            command=self.speak
        )
        submit_button.grid(row=0, column=0, padx=25)

        # Create Clear Button
        clear_button = customtkinter.CTkButton(
            button_frame,
            text="Clear",
            command=self.clear
        )
        clear_button.grid(row=0, column=1, padx=35)

    def __init__(self) -> None:
        self.app = customtkinter.CTk()
        self.app.title("Veronica")
        self.app.geometry('1000x1000')

        self.cache = redis.Redis(
            host='0.0.0.0', 
            port=6379, 
            db=0
        )

        self.frame = None

        self.my_text = None
        self.chat_entry = None
        self.camera = None

        customtkinter.set_appearance_mode("dark")
        customtkinter.set_default_color_theme("dark-blue")

        self.construct_ui()
        
        self.chat_entry.bind("<Return>", lambda x: self.speak())
        self.my_text.insert(END, "Conversation begins here!\n\n")
        self.update_frame()
