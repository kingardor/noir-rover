import whisper
from typing import Tuple

class WhisperSTT:
    def __init__(
            self, 
            model: str = 'models/small.pt', 
            english: bool = False, 
            translate: bool = False
        ) -> None:
        
        self.english = english
        self.translate = translate
        print("\033[96mLoading Whisper Model..\033[0m", end='', flush=True)
        self.model = whisper.load_model(f'{model}')
        print("\033[90m Done.\033[0m", flush=True)

    def inference(self) -> Tuple[str, str]:
        audio = whisper.load_audio('dictate.wav')
        audio = whisper.pad_or_trim(audio)
        mel = whisper.log_mel_spectrogram(audio).to(self.model.device)
        
        _, probs = self.model.detect_language(mel)
        lang = max(probs, key=probs.get)
        
        options = whisper.DecodingOptions(task="translate", language=lang)
        result = whisper.decode(self.model, mel, options)
        return lang, result.text