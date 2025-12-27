"""
Whisper Integration Module for Vision-Language-Action (VLA) System

This module handles the integration with OpenAI Whisper for speech-to-text conversion.
"""
import openai
import speech_recognition as sr
import io
import wave
from typing import Optional, Dict, Any
from .models.voice_command import VoiceCommand
import time
import logging

logger = logging.getLogger(__name__)


class WhisperIntegration:
    """
    Handles integration with OpenAI Whisper for speech-to-text conversion
    """

    def __init__(self, api_key: Optional[str] = None, model: str = "whisper-1"):
        """
        Initialize the Whisper integration

        Args:
            api_key: OpenAI API key (if not set via environment variable)
            model: Whisper model to use (default: "whisper-1")
        """
        if api_key:
            openai.api_key = api_key
        self.model = model
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

    def transcribe_audio_file(self, audio_file_path: str, language: str = "en") -> VoiceCommand:
        """
        Transcribe an audio file using Whisper

        Args:
            audio_file_path: Path to the audio file
            language: Language of the audio (default: "en")

        Returns:
            VoiceCommand object with the transcription
        """
        try:
            with open(audio_file_path, "rb") as audio_file:
                transcript = openai.Audio.transcribe(
                    self.model,
                    audio_file,
                    language=language,
                    response_format="verbose_json"
                )

            return VoiceCommand.create_from_audio(
                transcript=transcript.text,
                confidence=transcript.get("confidence", 0.8),  # Default confidence if not provided
                language=language,
                audio_data=audio_file_path
            )
        except Exception as e:
            logger.error(f"Error transcribing audio file {audio_file_path}: {e}")
            raise

    def transcribe_audio_data(self, audio_data: bytes, language: str = "en") -> VoiceCommand:
        """
        Transcribe raw audio data using Whisper

        Args:
            audio_data: Raw audio data
            language: Language of the audio (default: "en")

        Returns:
            VoiceCommand object with the transcription
        """
        try:
            # Create an in-memory file-like object
            audio_file = io.BytesIO(audio_data)

            # Use OpenAI's API to transcribe
            transcript = openai.Audio.transcribe(
                self.model,
                audio_file,
                language=language,
                response_format="verbose_json"
            )

            return VoiceCommand.create_from_audio(
                transcript=transcript.text,
                confidence=transcript.get("confidence", 0.8),
                language=language
            )
        except Exception as e:
            logger.error(f"Error transcribing audio data: {e}")
            raise

    def capture_and_transcribe(self, timeout: float = 5.0, language: str = "en") -> Optional[VoiceCommand]:
        """
        Capture audio from microphone and transcribe using Whisper

        Args:
            timeout: Maximum time to wait for audio input (default: 5.0 seconds)
            language: Language of the audio (default: "en")

        Returns:
            VoiceCommand object with the transcription, or None if no audio detected
        """
        try:
            with self.microphone as source:
                logger.info("Listening for audio...")
                # Listen for audio with timeout
                audio = self.recognizer.listen(source, timeout=timeout)

            # Save audio to temporary buffer
            audio_data = io.BytesIO()
            wav_writer = wave.open(audio_data, 'wb')
            # Note: This is a simplified approach - in practice, you'd use the raw audio data directly
            # For Whisper API, we need to save it in a proper format
            raw_data = audio.get_raw_data()
            wav_writer.setnchannels(audio.channels)
            wav_writer.setsampwidth(audio.sample_width)
            wav_writer.setframerate(audio.sample_rate)
            wav_writer.writeframes(raw_data)
            wav_writer.close()

            # Reset buffer position
            audio_data.seek(0)

            # Use OpenAI's API to transcribe
            transcript = openai.Audio.transcribe(
                self.model,
                audio_data,
                language=language,
                response_format="verbose_json"
            )

            return VoiceCommand.create_from_audio(
                transcript=transcript.text,
                confidence=transcript.get("confidence", 0.8),
                language=language
            )
        except sr.WaitTimeoutError:
            logger.info("No audio detected within timeout period")
            return None
        except Exception as e:
            logger.error(f"Error in voice capture and transcription: {e}")
            raise

    def transcribe_with_fallback(self, audio_source: str, language: str = "en") -> VoiceCommand:
        """
        Transcribe audio with fallback to local speech recognition if Whisper fails

        Args:
            audio_source: Either a file path or a flag to indicate microphone input
            language: Language of the audio (default: "en")

        Returns:
            VoiceCommand object with the transcription
        """
        try:
            # Try Whisper first
            if audio_source == "microphone":
                result = self.capture_and_transcribe(language=language)
            else:
                result = self.transcribe_audio_file(audio_source, language=language)

            if result and result.confidence > 0.7:
                return result
        except Exception as e:
            logger.warning(f"Whisper failed: {e}, trying fallback...")

        # Fallback to local speech recognition
        return self._fallback_recognition(audio_source, language)

    def _fallback_recognition(self, audio_source: str, language: str = "en") -> VoiceCommand:
        """
        Fallback speech recognition using local libraries

        Args:
            audio_source: Either a file path or a flag to indicate microphone input
            language: Language of the audio

        Returns:
            VoiceCommand object with the transcription
        """
        try:
            if audio_source == "microphone":
                with self.microphone as source:
                    logger.info("Using fallback: Listening for audio...")
                    audio = self.recognizer.listen(source, timeout=5.0)
                    transcript = self.recognizer.recognize_google(audio, language=language)
            else:
                with sr.AudioFile(audio_source) as source:
                    audio = self.recognizer.record(source)
                    transcript = self.recognizer.recognize_google(audio, language=language)

            # Lower confidence for fallback recognition
            return VoiceCommand.create_from_audio(
                transcript=transcript,
                confidence=0.6,  # Lower confidence for local recognition
                language=language,
                audio_data=audio_source if audio_source != "microphone" else None
            )
        except Exception as e:
            logger.error(f"Fallback recognition also failed: {e}")
            raise Exception("Both Whisper and fallback recognition failed")


# Example usage:
if __name__ == "__main__":
    import os

    # Initialize Whisper integration
    # Note: In a real system, you would set OPENAI_API_KEY environment variable
    whisper = WhisperIntegration()

    # Example 1: Transcribe from microphone (uncomment to test)
    # command = whisper.capture_and_transcribe()
    # if command:
    #     print(f"Transcribed: {command.transcript}")
    #     print(f"Confidence: {command.confidence}")

    # Example 2: Transcribe from audio file (this would require an actual audio file)
    # try:
    #     command = whisper.transcribe_audio_file("path/to/audio.wav")
    #     print(f"File transcription: {command.transcript}")
    # except Exception as e:
    #     print(f"File transcription failed: {e}")

    # Example 3: Show the class structure
    print("WhisperIntegration class ready for use with OpenAI Whisper API")
    print("Requires OPENAI_API_KEY environment variable to be set")