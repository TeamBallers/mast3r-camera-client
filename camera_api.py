#!/usr/bin/env python3
"""
API server to receive images via POST requests and queue them for SLAM processing.

Usage:
    # With continuous processing
    python image_receiver_api.py --continuous

    # Batch mode (process manually)
    python image_receiver_api.py
"""

import argparse
import io
import subprocess
import threading
import time
from pathlib import Path
from datetime import datetime
from typing import Optional
import glob
import shutil

from fastapi import FastAPI, File, UploadFile, BackgroundTasks
from fastapi.responses import JSONResponse
from PIL import Image
import uvicorn

app = FastAPI(title="Camera API Server")

camera_process = None  # global reference

def run_camera_client ():
    """Run the camera client as a subprocess."""
    print("Starting camera client...")
    # Adjust the command as needed to run your camera client
    cmd = ["python", "camera_client.py", "--host", "100.101.179.51", "--port", "5050", "--fps", "100"]
    global camera_process
    camera_process = subprocess.Popen(cmd)

@app.post("/start")
async def start_task(background_tasks: BackgroundTasks):
    background_tasks.add_task(run_camera_client)
    return JSONResponse(content={
        "status": "success",
        "message": "Camera client started in background"
    })

@app.post("/stop")
async def stop_task():
    global camera_process
    if camera_process is not None:
        camera_process.terminate()
        
        try:
            camera_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            camera_process.kill()
        camera_process = None

        return JSONResponse(content={
            "status": "success",
            "message": "Camera client stopped"
        })
    else:
        return JSONResponse(content={
            "status": "error",
            "message": "Camera client is not running"
        }, status_code=400)
