git fetch upstream
git merge upstream/master

cd NuttX
git pull
cd ..

read -p "Press Y to continue with build..."  continue

if [ ${continue^^} = Y ] 
then
make distclean
make archives -j8
make -j8
else
echo "Exiting..."
fi

