"""Read-only acquisition jobs and a path-confined recording library."""
from __future__ import annotations

import json
from pathlib import Path
import re
import threading
import time
import uuid

from ..recording import JsonlRecorder, observation_to_dict
from ..replay import RecordingReader, inspect_recording


class RecordingLibrary:
    def __init__(self, client, root: Path):
        self.client, self.root = client, root.resolve()
        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._active = None
        self._closed = False

    def _path(self, identifier, suffix):
        if not re.fullmatch(r"[0-9a-f]{32}", identifier):
            raise ValueError("Invalid recording ID")
        path = self.root / f"{identifier}.{suffix}"
        if path.is_symlink() or path.resolve().parent != self.root:
            raise ValueError("Recording path must remain inside library")
        return path

    def _save(self, job):
        path = self._path(job["id"], "json")
        temporary = self._path(job["id"], "tmp")
        temporary.write_text(json.dumps(job, ensure_ascii=False, allow_nan=False), encoding="utf-8")
        temporary.replace(path)

    def get(self, identifier):
        with self._lock:
            job = json.loads(self._path(identifier, "json").read_text(encoding="utf-8"))
            if job["status"] in {"recording", "stopping"} and identifier != self._active:
                job["status"] = "interrupted"
            return job

    def list(self):
        jobs = [self.get(path.stem) for path in self.root.glob("*.json")
                if re.fullmatch(r"[0-9a-f]{32}", path.stem)]
        return sorted(jobs, key=lambda job: job["created_unix_ns"], reverse=True)

    def start(self, options):
        with self._lock:
            if self._closed or self._active is not None:
                raise RuntimeError("Recording service closed or another recording is active")
            self.root.mkdir(parents=True, exist_ok=True)
            identifier = uuid.uuid4().hex
            job = {"id": identifier, "status": "recording", "created_unix_ns": time.time_ns(),
                   "options": options, "motion_verified": False, "tactile_contact_verified": False}
            self._save(job)
            self._stop.clear()
            self._active = identifier
            self._thread = threading.Thread(target=self._run, args=(job,), daemon=True)
            self._thread.start()
            return dict(job)

    def _run(self, job):
        try:
            count = JsonlRecorder(self.client, self._path(job["id"], "jsonl")).run(
                **job["options"], stop_event=self._stop)
            job.update(status="stopped" if self._stop.is_set() else "completed", samples=count)
            job["inspection"] = inspect_recording(self._path(job["id"], "jsonl"))
        except Exception as exc:
            job.update(status="failed", error=f"{type(exc).__name__}: {exc}")
        finally:
            job["finished_unix_ns"] = time.time_ns()
            with self._lock:
                try:
                    self._save(job)
                finally:
                    self._active = None

    def stop(self, identifier):
        with self._lock:
            if identifier != self._active:
                raise ValueError("Recording is not active")
            self._stop.set()
        return {"id": identifier, "status": "stopping"}

    def path(self, identifier):
        job = self.get(identifier)
        if job["status"] in {"recording", "stopping"}:
            raise RuntimeError("Stop acquisition before reading its evidence file")
        path = self._path(identifier, "jsonl")
        if not path.is_file():
            raise FileNotFoundError(identifier)
        return path

    def samples(self, identifier, offset=0, limit=20):
        reader = RecordingReader(self.path(identifier))
        samples = []
        for sample in reader:
            if sample.index < offset:
                continue
            samples.append({"index": sample.index, "observation": observation_to_dict(sample.observation, False),
                            "omitted_images": list(sample.omitted_images), "camera_metadata": sample.camera_metadata})
            if len(samples) >= limit:
                break
        return {"source": "recording", "id": identifier, "metadata": reader.metadata, "samples": samples,
                "motion_verified": False, "executed_actions": 0}

    def close(self):
        with self._lock:
            self._closed = True
            self._stop.set()
            thread = self._thread
        if thread is not None:
            thread.join(timeout=10)
            if thread.is_alive():
                raise RuntimeError("Acquisition did not stop within 10 s; backend read still pending")
