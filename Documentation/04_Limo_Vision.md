# **3. Limo Vision**

The Limo is equipped with a Jetson Nano computer. This PC is designed for doing advanced parallel processing such as Deep Learning. We’re going to use these capabilities to find objects and better understand the environment around the robot

This package has to be **cloned and compiled on the real robot**. This needs a GPU computer to run!
- You will need to clone and build the vision package.
````shell
git clone --recursive http://github.com/limo-agx/limo_vision.git
cd ..
rosdep install --from-paths src --ignore-src -y
catkin_make
````