pip install serial




# Display a pause message
echo "Press any key to continue after manually installing the rest of configuration."
echo "[PAUSE]"
# Pause the script until the user presses a key
read -n 1 -s -r -p "Press any key to continue..."

# Here you can insert the code for manually installing the libraries, if needed

# Continue with the rest of the script
echo ""
echo "[CONTINUE] Make sure you have installed and configured all."



echo "when connected and configured: "


echo "You will see the /dev/ttyACM0 has been created automatically."
ls /dev/tty*

echo "display the raw data stream from the RTK Reciever located at the /dev/ttyACM0 serial port."
sudo cat /dev/ttyACM0