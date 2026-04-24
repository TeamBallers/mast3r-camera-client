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
import subprocess, sys, os

from fastapi import FastAPI, BackgroundTasks
from fastapi.responses import JSONResponse
import uvicorn

app = FastAPI(title="Camera API Server")

camera_process = None  
camera_log_file = None

destination_ip = "100.101.179.51"
destination_port = 5050
fps = 100

def run_camera_client ():
    """Run the camera client as a subprocess."""
    print("Starting camera client...")
    global destination_ip, destination_port, fps, camera_process

    log_file = open("/tmp/camera_client.log", "a")  # append so you can tail it
    global camera_log_file
    camera_log_file = log_file

    cmd = [
        sys.executable,           # use the same python that's running the API
        "camera_client.py",
        "--host", destination_ip,
        "--port", str(destination_port),
        "--fps", str(fps)
    ]

    camera_process = subprocess.Popen(
        cmd,
        stdout=log_file,
        stderr=log_file,
        start_new_session=True,    # detach from uvicorn's process group
        close_fds=True,
    )

@app.post("/start")
async def start_task(background_tasks: BackgroundTasks):
    if camera_process is not None and camera_process.poll() is None:
        return JSONResponse(content={
            "status": "error",
            "message": "Camera client is already running"
        }, status_code=400)
    
    background_tasks.add_task(run_camera_client)
    return JSONResponse(content={
        "status": "success",
        "message": "Camera client started in background"
    })

@app.post("/stop")
async def stop_task():
    global camera_process, camera_log_file
    if camera_process is not None:
        camera_process.terminate()
        
        try:
            camera_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            camera_process.kill()
        camera_process = None

        if camera_log_file:
            camera_log_file.close()
            camera_log_file = None

        return JSONResponse(content={
            "status": "success",
            "message": "Camera client stopped"
        })
    else:
        return JSONResponse(content={
            "status": "error",
            "message": "Camera client is not running"
        }, status_code=400)
    
def main():
    global destination_ip, destination_port, fps

    parser = argparse.ArgumentParser(description="Camera API")
    parser.add_argument(
        "--dest_ip",
        type=str,
        default="100.101.179.51",
        help="IP to upload images to (default: 100.101.179.51)"
    )
    parser.add_argument(
        "--dest_port",
        type=int,
        default=5050,
        help="Port to upload images to (default: 5050)"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5000,
        help="Port to run on (default: 5000)"
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=100,
        help="FPS for camera client (default: 100)"
    )

    args = parser.parse_args()

    destination_ip = args.dest_ip
    destination_port = args.dest_port
    fps = args.fps

    print(f"🌐 Starting server on http://0.0.0.0:{args.port}")
    print("\nExample usage:")
    print(f"  curl -X POST http://localhost:{args.port}/start")
    print(f"  curl -X POST http://localhost:{args.port}/stop")

    uvicorn.run(app, host="0.0.0.0", port=args.port)


if __name__ == "__main__":
    main()
