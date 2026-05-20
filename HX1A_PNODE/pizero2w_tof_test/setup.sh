#!/bin/bash
set -e

echo "=== HX1A ToF Camera Setup ==="

echo "[1/4] Installing system packages..."
echo 'password67' | sudo -S apt update && sudo apt install -y python3-opencv python3-numpy python3-flask

echo "[2/4] Creating venv with system packages..."
python3 -m venv --system-site-packages venv

echo "[3/4] Installing ArducamDepthCamera..."
source venv/bin/activate && pip install ArducamDepthCamera

echo "[4/4] Installing systemd service..."
cat > /tmp/hx1a_tof.service << 'EOF'
[Unit]
Description=HX1A ToF Camera Web Server
After=network.target

[Service]
Type=simple
User=nao
WorkingDirectory=/home/nao/HX1A_PNODE/HX1A_PNODE/pizero2w_tof_test
ExecStart=/home/nao/HX1A_PNODE/HX1A_PNODE/pizero2w_tof_test/venv/bin/python3 app.py --host 0.0.0.0 --port 5000
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo 'password67' | sudo -S mv /tmp/hx1a_tof.service /etc/systemd/system/
echo 'password67' | sudo -S systemctl daemon-reload
echo 'password67' | sudo -S systemctl enable hx1a_tof
echo 'password67' | sudo -S systemctl start hx1a_tof

echo "=== Done! Access at http://$(hostname -I | awk '{print $1}'):5000 ==="
