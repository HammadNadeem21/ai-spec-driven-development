"""
Whisper Speech-to-Text Processor for Vision-Language-Action (VLA) System

This module processes audio through Whisper for speech-to-text conversion.
"""
import openai
import io
import wave
import numpy as np
from typing import Optional, Dict, Any, Callable
from .models.voice_command import VoiceCommand
from .whisper_integration import WhisperIntegration
import logging
import time

logger = logging.getLogger(__name__)


class WhisperProcessor:
    """
    Processes audio through Whisper for speech-to-text conversion
    """

    def __init__(self, whisper_integration: WhisperIntegration, language: str = "en"):
        """
        Initialize the Whisper processor

        Args:
            whisper_integration: Instance of WhisperIntegration for API calls
            language: Default language for speech recognition (default: "en")
        """
        self.whisper_integration = whisper_integration
        self.language = language
        self.processing_stats = {
            "total_processed": 0,
            "successful_conversions": 0,
            "failed_conversions": 0,
            "total_processing_time": 0.0
        }

    def process_audio_data(self, audio_data: bytes, language: Optional[str] = None) -> VoiceCommand:
        """
        Process raw audio data through Whisper

        Args:
            audio_data: Raw audio data to process
            language: Language of the audio (optional, uses default if not provided)

        Returns:
            VoiceCommand object with the transcription
        """
        start_time = time.time()

        try:
            # Use provided language or default
            proc_language = language or self.language

            # Process with Whisper integration
            voice_command = self.whisper_integration.transcribe_audio_data(
                audio_data,
                language=proc_language
            )

            # Update stats
            self.processing_stats["total_processed"] += 1
            self.processing_stats["successful_conversions"] += 1
            self.processing_stats["total_processing_time"] += (time.time() - start_time)

            logger.info(f"Successfully processed audio: '{voice_command.transcript[:50]}...' "
                       f"(confidence: {voice_command.confidence:.2f})")

            return voice_command

        except Exception as e:
            self.processing_stats["total_processed"] += 1
            self.processing_stats["failed_conversions"] += 1
            self.processing_stats["total_processing_time"] += (time.time() - start_time)

            logger.error(f"Error processing audio data: {e}")
            raise

    def process_audio_file(self, file_path: str, language: Optional[str] = None) -> VoiceCommand:
        """
        Process audio file through Whisper

        Args:
            file_path: Path to the audio file to process
            language: Language of the audio (optional, uses default if not provided)

        Returns:
            VoiceCommand object with the transcription
        """
        start_time = time.time()

        try:
            # Use provided language or default
            proc_language = language or self.language

            # Process with Whisper integration
            voice_command = self.whisper_integration.transcribe_audio_file(
                file_path,
                language=proc_language
            )

            # Update stats
            self.processing_stats["total_processed"] += 1
            self.processing_stats["successful_conversions"] += 1
            self.processing_stats["total_processing_time"] += (time.time() - start_time)

            logger.info(f"Successfully processed file {file_path}: '{voice_command.transcript[:50]}...' "
                       f"(confidence: {voice_command.confidence:.2f})")

            return voice_command

        except Exception as e:
            self.processing_stats["total_processed"] += 1
            self.processing_stats["failed_conversions"] += 1
            self.processing_stats["total_processing_time"] += (time.time() - start_time)

            logger.error(f"Error processing audio file {file_path}: {e}")
            raise

    def process_realtime_audio(self, audio_generator, language: Optional[str] = None,
                              callback: Optional[Callable[[VoiceCommand], None]] = None) -> None:
        """
        Process audio in real-time from a generator (e.g., from audio capture)

        Args:
            audio_generator: Generator that yields audio data chunks
            language: Language of the audio (optional, uses default if not provided)
            callback: Optional callback function to handle transcriptions
        """
        proc_language = language or self.language

        for audio_chunk in audio_generator:
            try:
                # Process the audio chunk
                voice_command = self.process_audio_data(audio_chunk, proc_language)

                # Call the callback if provided
                if callback:
                    try:
                        callback(voice_command)
                    except Exception as e:
                        logger.error(f"Error in callback function: {e}")

            except Exception as e:
                logger.error(f"Error processing audio chunk in real-time: {e}")
                # Continue processing other chunks despite errors

    def process_with_fallback(self, audio_data: bytes, language: Optional[str] = None) -> VoiceCommand:
        """
        Process audio with fallback to local speech recognition if Whisper fails

        Args:
            audio_data: Raw audio data to process
            language: Language of the audio (optional, uses default if not provided)

        Returns:
            VoiceCommand object with the transcription
        """
        start_time = time.time()

        try:
            # Use provided language or default
            proc_language = language or self.language

            # Process with Whisper integration (with fallback)
            voice_command = self.whisper_integration.transcribe_with_fallback(
                "audio_data",  # Indicate we're passing audio data
                language=proc_language
            )

            # Update stats
            self.processing_stats["total_processed"] += 1
            self.processing_stats["successful_conversions"] += 1
            self.processing_stats["total_processing_time"] += (time.time() - start_time)

            logger.info(f"Successfully processed audio with fallback: '{voice_command.transcript[:50]}...' "
                       f"(confidence: {voice_command.confidence:.2f})")

            return voice_command

        except Exception as e:
            self.processing_stats["total_processed"] += 1
            self.processing_stats["failed_conversions"] += 1
            self.processing_stats["total_processing_time"] += (time.time() - start_time)

            logger.error(f"Error processing audio with fallback: {e}")
            raise

    def validate_transcription_quality(self, voice_command: VoiceCommand,
                                     min_confidence: float = 0.7) -> Dict[str, Any]:
        """
        Validate the quality of a transcription

        Args:
            voice_command: VoiceCommand to validate
            min_confidence: Minimum confidence threshold (default: 0.7)

        Returns:
            Dictionary with validation results
        """
        validation_results = {
            "is_valid": True,
            "issues": [],
            "confidence_ok": voice_command.confidence >= min_confidence,
            "text_quality": self._assess_text_quality(voice_command.transcript)
        }

        # Check confidence
        if voice_command.confidence < min_confidence:
            validation_results["is_valid"] = False
            validation_results["issues"].append(f"Confidence {voice_command.confidence} below threshold {min_confidence}")

        # Check for empty transcription
        if not voice_command.transcript.strip():
            validation_results["is_valid"] = False
            validation_results["issues"].append("Transcription is empty")

        # Check for common Whisper artifacts
        if self._has_common_artifacts(voice_command.transcript):
            validation_results["issues"].append("Transcription contains common Whisper artifacts")

        return validation_results

    def _assess_text_quality(self, text: str) -> Dict[str, Any]:
        """
        Assess the quality of transcribed text

        Args:
            text: Transcribed text to assess

        Returns:
            Dictionary with text quality metrics
        """
        if not text:
            return {"readability": 0.0, "completeness": 0.0, "grammar_like": 0.0}

        # Simple readability metrics
        words = text.split()
        word_count = len(words)
        avg_word_length = np.mean([len(word) for word in words]) if words else 0

        # Sentence structure (very basic)
        has_punctuation = any(p in text for p in '.!?')
        has_uppercase = any(c.isupper() for c in text[:10])  # Check beginning

        return {
            "word_count": word_count,
            "avg_word_length": avg_word_length,
            "has_punctuation": has_punctuation,
            "has_uppercase": has_uppercase,
            "readability_score": min(1.0, word_count / 50)  # Very basic score
        }

    def _has_common_artifacts(self, text: str) -> bool:
        """
        Check if transcription has common Whisper artifacts

        Args:
            text: Transcribed text to check

        Returns:
            True if artifacts are detected, False otherwise
        """
        # Common Whisper artifacts to detect
        artifacts = [
            "uh", "um", "er", "mm", "hmm",  # Fillers that might be over-transcribed
            "...",  # Excessive ellipses
            "you know", "i mean", "right"  # Common phrases that might be artifacts
        ]

        text_lower = text.lower()
        for artifact in artifacts:
            if artifact in text_lower:
                # Only count as artifact if it appears multiple times or in context
                # that suggests it's noise rather than intentional speech
                if text_lower.count(artifact) > 1:
                    return True

        return False

    def get_processing_stats(self) -> Dict[str, Any]:
        """
        Get statistics about audio processing performance

        Returns:
            Dictionary with processing statistics
        """
        stats = self.processing_stats.copy()
        if stats["total_processed"] > 0:
            stats["success_rate"] = stats["successful_conversions"] / stats["total_processed"]
            stats["avg_processing_time"] = stats["total_processing_time"] / stats["total_processed"]
        else:
            stats["success_rate"] = 0.0
            stats["avg_processing_time"] = 0.0

        return stats

    def reset_stats(self) -> None:
        """
        Reset processing statistics
        """
        self.processing_stats = {
            "total_processed": 0,
            "successful_conversions": 0,
            "failed_conversions": 0,
            "total_processing_time": 0.0
        }


# Example usage:
if __name__ == "__main__":
    import os

    # This example shows how to use the WhisperProcessor
    # In a real system, you would need to initialize WhisperIntegration with API key
    print("WhisperProcessor class ready for use")
    print("Requires WhisperIntegration instance with OpenAI API configuration")

    # Example of how it would be used:
    # whisper_integration = WhisperIntegration()  # Requires API key
    # processor = WhisperProcessor(whisper_integration)
    #
    # # Process some audio data
    # # audio_data = b"..."  # Raw audio bytes
    # # command = processor.process_audio_data(audio_data)
    # # print(f"Transcription: {command.transcript}")
    # # print(f"Confidence: {command.confidence}")