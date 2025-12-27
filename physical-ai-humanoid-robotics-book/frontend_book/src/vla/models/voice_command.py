"""
Voice Command Model for Vision-Language-Action (VLA) System

Based on the data model defined in the VLA specification.
"""
from dataclasses import dataclass
from typing import Optional
import time
import uuid


@dataclass
class VoiceCommand:
    """
    Represents a voice command captured from the user

    Fields:
    - id: Unique identifier for the command
    - audio_data: Raw audio data or reference to audio file
    - transcript: Text transcript of the voice command
    - confidence: Confidence score of the speech-to-text conversion
    - timestamp: When the command was captured
    - language: Detected language of the command
    """

    id: str
    transcript: str
    confidence: float
    timestamp: str
    language: str
    audio_data: Optional[str] = None  # Reference to audio file or None if not stored

    def __post_init__(self):
        """Validate the VoiceCommand instance after initialization"""
        if not self.transcript.strip():
            raise ValueError("Transcript must not be empty")

        if not (0.0 <= self.confidence <= 1.0):
            raise ValueError("Confidence must be between 0.0 and 1.0")

    @classmethod
    def create_from_audio(cls, transcript: str, confidence: float, language: str = "en", audio_data: Optional[str] = None):
        """
        Create a VoiceCommand instance from audio processing results

        Args:
            transcript: The text transcript of the voice command
            confidence: Confidence score of the speech-to-text conversion
            language: Detected language of the command (default: "en")
            audio_data: Raw audio data or reference to audio file
        """
        return cls(
            id=f"vc_{uuid.uuid4().hex[:8]}",
            transcript=transcript,
            confidence=confidence,
            timestamp=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            language=language,
            audio_data=audio_data
        )


# Example usage:
if __name__ == "__main__":
    # Create a sample voice command
    command = VoiceCommand.create_from_audio(
        transcript="Go to the kitchen and bring me a cup of water",
        confidence=0.92,
        language="en"
    )

    print(f"Voice Command ID: {command.id}")
    print(f"Transcript: {command.transcript}")
    print(f"Confidence: {command.confidence}")
    print(f"Timestamp: {command.timestamp}")
    print(f"Language: {command.language}")