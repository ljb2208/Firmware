git fetch upstream
git merge upstream/master

read -p "Press Y to continue with build..."  continue

if [ ${continue^^} = Y ] 
then
make configure_px4io
make clean
make
make configure_px4fmu
make clean
make
else
echo "Exiting..."
fi

