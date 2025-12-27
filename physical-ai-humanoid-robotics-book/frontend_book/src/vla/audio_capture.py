"""
Audio Capture Module for Vision-Language-Action (VLA) System

This module handles audio capture from various sources for the voice command pipeline.
"""
import pyaudio
import wave
import numpy as np
import time
from typing import Optional, Callable, Dict, Any
import threading
import queue
import logging

logger = logging.getLogger(__name__)


class AudioCapture:
    """
    Handles audio capture from microphone or other audio sources
    """

    def __init__(self, sample_rate: int = 16000, chunk_size: int = 1024, channels: int = 1):
        """
        Initialize the audio capture system

        Args:
            sample_rate: Audio sample rate (default: 16000 Hz)
            chunk_size: Size of audio chunks (default: 1024)
            channels: Number of audio channels (default: 1 for mono)
        """
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.channels = channels
        self.audio = pyaudio.PyAudio()

        # Audio stream
        self.stream = None
        self.is_recording = False

        # Audio data buffer
        self.audio_queue = queue.Queue()
        self.recording_thread = None

        # Configuration
        self.energy_threshold = 4000  # Adjusted for ambient noise
        self.dynamic_energy_threshold = True
        self.pause_threshold = 0.8  # Seconds of silence after which to stop listening
        self.phrase_threshold = 0.3  # Minimum seconds of speaking before considering phrase complete

    def start_recording(self, callback: Optional[Callable[[bytes], None]] = None) -> None:
        """
        Start recording audio from the microphone

        Args:
            callback: Optional callback function to process audio chunks
        """
        if self.is_recording:
            logger.warning("Recording is already in progress")
            return

        try:
            # Open audio stream
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )

            self.is_recording = True

            # Start recording thread
            self.recording_thread = threading.Thread(
                target=self._recording_worker,
                args=(callback,)
            )
            self.recording_thread.daemon = True
            self.recording_thread.start()

            logger.info("Started audio recording")

        except Exception as e:
            logger.error(f"Failed to start audio recording: {e}")
            raise

    def stop_recording(self) -> None:
        """
        Stop recording audio
        """
        if not self.is_recording:
            logger.warning("Recording is not in progress")
            return

        self.is_recording = False

        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None

        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join(timeout=1.0)

        logger.info("Stopped audio recording")

    def _recording_worker(self, callback: Optional[Callable[[bytes], None]] = None) -> None:
        """
        Worker thread for capturing audio data

        Args:
            callback: Optional callback function to process audio chunks
        """
        while self.is_recording:
            try:
                # Read audio data from stream
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)

                # Add to queue for other consumers
                self.audio_queue.put(data)

                # Call callback if provided
                if callback:
                    try:
                        callback(data)
                    except Exception as e:
                        logger.error(f"Error in audio callback: {e}")

            except Exception as e:
                logger.error(f"Error in recording worker: {e}")
                break

    def capture_audio_chunk(self, duration: float = 1.0) -> bytes:
        """
        Capture a single chunk of audio for a specified duration

        Args:
            duration: Duration to capture audio in seconds (default: 1.0)

        Returns:
            Raw audio data as bytes
        """
        if self.is_recording:
            logger.warning("Cannot capture audio chunk while continuous recording is active")
            return b""

        frames = []
        try:
            # Open temporary stream
            temp_stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )

            # Calculate number of chunks needed
            chunks_needed = int(self.sample_rate * duration / self.chunk_size)

            for _ in range(chunks_needed):
                data = temp_stream.read(self.chunk_size, exception_on_overflow=False)
                frames.append(data)

            # Close temporary stream
            temp_stream.stop_stream()
            temp_stream.close()

            # Combine frames into single bytes object
            audio_data = b"".join(frames)
            return audio_data

        except Exception as e:
            logger.error(f"Error capturing audio chunk: {e}")
            raise

    def capture_phrase(self, timeout: float = 5.0, phrase_time_limit: Optional[float] = None) -> Optional[bytes]:
        """
        Capture a phrase of speech (stops after pause_threshold seconds of silence)

        Args:
            timeout: Maximum time to wait for speech (default: 5.0 seconds)
            phrase_time_limit: Maximum time for a single phrase (default: None)

        Returns:
            Raw audio data as bytes, or None if timeout occurs
        """
        if self.is_recording:
            logger.warning("Cannot capture phrase while continuous recording is active")
            return None

        frames = []
        try:
            # Open temporary stream
            temp_stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )

            # Wait for speech to begin
            logger.info("Listening for speech...")

            # Energy threshold for detecting speech
            speech_detected = False
            silence_chunks = 0
            max_silence_chunks = int(self.sample_rate / self.chunk_size * self.pause_threshold)

            start_time = time.time()

            while self.is_recording:
                if timeout and (time.time() - start_time) > timeout:
                    logger.info("Timeout waiting for speech")
                    temp_stream.stop_stream()
                    temp_stream.close()
                    return None

                data = temp_stream.read(self.chunk_size, exception_on_overflow=False)
                frames.append(data)

                # Calculate audio energy
                audio_array = np.frombuffer(data, dtype=np.int16)
                energy = np.sum(np.abs(audio_array.astype(np.float32))) / len(audio_array)

                if energy > self.energy_threshold:
                    speech_detected = True
                    silence_chunks = 0  # Reset silence counter
                elif speech_detected:  # Only count silence after speech is detected
                    silence_chunks += 1

                # If we've detected speech and enough silence has passed, stop
                if speech_detected and silence_chunks > max_silence_chunks:
                    logger.info("Speech phrase detected and captured")
                    break

            # Close temporary stream
            temp_stream.stop_stream()
            temp_stream.close()

            # Combine frames into single bytes object
            if frames:
                audio_data = b"".join(frames)
                return audio_data
            else:
                return None

        except Exception as e:
            logger.error(f"Error capturing phrase: {e}")
            raise

    def save_audio_to_file(self, audio_data: bytes, filename: str) -> None:
        """
        Save raw audio data to a WAV file

        Args:
            audio_data: Raw audio data to save
            filename: Name of the file to save to
        """
        try:
            with wave.open(filename, 'wb') as wf:
                wf.setnchannels(self.channels)
                wf.setsampwidth(self.audio.get_sample_size(pyaudio.paInt16))
                wf.setframerate(self.sample_rate)
                wf.writeframes(audio_data)

            logger.info(f"Audio saved to {filename}")
        except Exception as e:
            logger.error(f"Error saving audio to file {filename}: {e}")
            raise

    def get_audio_from_queue(self, timeout: Optional[float] = None) -> Optional[bytes]:
        """
        Get audio data from the internal queue

        Args:
            timeout: Timeout in seconds (default: None, blocking)

        Returns:
            Raw audio data as bytes, or None if timeout
        """
        try:
            return self.audio_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def adjust_for_ambient_noise(self, duration: float = 1.0) -> None:
        """
        Adjust the energy threshold based on ambient noise levels

        Args:
            duration: Duration to analyze ambient noise (default: 1.0 seconds)
        """
        if not self.dynamic_energy_threshold:
            return

        try:
            # Open temporary stream
            temp_stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )

            logger.info("Adjusting for ambient noise...")

            energy_readings = []
            chunks_needed = int(self.sample_rate * duration / self.chunk_size)

            for _ in range(chunks_needed):
                data = temp_stream.read(self.chunk_size, exception_on_overflow=False)
                audio_array = np.frombuffer(data, dtype=np.int16)
                energy = np.sum(np.abs(audio_array.astype(np.float32))) / len(audio_array)
                energy_readings.append(energy)

            # Calculate average energy and set threshold
            avg_energy = np.mean(energy_readings)
            self.energy_threshold = avg_energy * 2  # Set threshold to 2x ambient level

            logger.info(f"Set energy threshold to {self.energy_threshold}")

            # Close temporary stream
            temp_stream.stop_stream()
            temp_stream.close()

        except Exception as e:
            logger.error(f"Error adjusting for ambient noise: {e}")
            raise

    def is_speech_detected(self, audio_data: bytes) -> bool:
        """
        Check if speech is detected in audio data

        Args:
            audio_data: Raw audio data to analyze

        Returns:
            True if speech is detected, False otherwise
        """
        try:
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            energy = np.sum(np.abs(audio_array.astype(np.float32))) / len(audio_array)
            return energy > self.energy_threshold
        except Exception as e:
            logger.error(f"Error detecting speech: {e}")
            return False

    def __del__(self):
        """
        Cleanup audio resources
        """
        try:
            if self.stream and not self.stream.is_stopped():
                self.stream.stop_stream()
                self.stream.close()
            self.audio.terminate()
        except:
            pass  # Ignore errors during cleanup


# Example usage:
if __name__ == "__main__":
    # Create an audio capture instance
    capture = AudioCapture()

    try:
        # Adjust for ambient noise
        capture.adjust_for_ambient_noise(duration=1.0)
        print(f"Energy threshold set to: {capture.energy_threshold}")

        # Capture a short phrase
        print("Please speak now...")
        audio_data = capture.capture_phrase(timeout=5.0)

        if audio_data:
            print(f"Captured {len(audio_data)} bytes of audio data")
            print(f"Speech detected: {capture.is_speech_detected(audio_data)}")

            # Save to file
            capture.save_audio_to_file(audio_data, "captured_audio.wav")
            print("Audio saved to captured_audio.wav")
        else:
            print("No speech detected")

    except Exception as e:
        print(f"Error in audio capture: {e}")
    finally:
        # Cleanup
        del capture