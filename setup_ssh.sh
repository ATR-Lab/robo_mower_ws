#!/bin/bash
# SSH Setup Script for Raspberry Pi Remote Access

echo "🔧 Setting up SSH access for remote development..."

# Install SSH server
sudo apt update
sudo apt install -y openssh-server

# Enable SSH service
sudo systemctl enable ssh
sudo systemctl start ssh

# Check SSH status
sudo systemctl status ssh

# Configure SSH for better security
sudo cp /etc/ssh/sshd_config /etc/ssh/sshd_config.backup

# Allow SSH connections (ensure it's enabled)
echo "SSH setup complete!"
echo "📡 Your Pi IP: $(hostname -I | cut -d' ' -f1)"
echo "🖥️ Your Pi hostname: $(hostname)"
echo ""
echo "From your PC, connect with:"
echo "ssh alwinsdon@$(hostname -I | cut -d' ' -f1)"
echo ""
echo "For VS Code Remote SSH:"
echo "1. Install 'Remote - SSH' extension"
echo "2. Connect to: alwinsdon@$(hostname -I | cut -d' ' -f1)"
