echo "before"

# Display a pause message
echo "Press any key to continue after manually installing the required libraries."
echo "[PAUSE]"
# Pause the script until the user presses a key
read -n 1 -s -r -p "Press any key to continue..."

# Here you can insert the code for manually installing the libraries, if needed

# Continue with the rest of the script
echo ""
echo "[CONTINUE] Continuation of the script after manual installation of libraries."

echo "after"



# Assicurati di sostituire 'file.rules' con il nome effettivo del tuo file .rules
file="resources/95-xsens-ftdi.rules"

# Verifica se il file esiste
if [ -f "$file" ]; then
    cat "$file"  # Stampare il contenuto del file
else
    echo "Il file $file non esiste."
    exit 1
fi
