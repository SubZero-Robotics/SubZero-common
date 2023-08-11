# Coral-based vision

Device has an IP address of `10.56.90.4` and uses NT3

## Vision Service

The vision service is a systemd service that autostarts the python script on the Coral Dev Board on boot. 

### The vision.service File

The vision.service file contains the command to run the vision python script. The file is located at `/etc/systemd/system/vision.service`

### Restarting the Service

If you made a change to the vision.service file make sure to reload the daemon and the vision script with these commands:

1. `sudo systemctl daemon-reload`
2. `sudo systemctl restart vision`

### Checking Status

To check the status of the vision service, use this command:
`sudo systemctl status vision`