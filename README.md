SLAM Alorigthm video link:

https://youtu.be/L3nrXk-GaX4


In this video left side is without SLAM algorithm(only odometry) and right side is with SLAM(alogorithm).
We can notice the loop closure in this video at timestamp between 1:26 to 1:31.

To use this code:

mkdir SLAM

cd SLAM

mkdir build

cd build

cmake ..

cd cui

./SLAM ~/SLAM/dataset/corridor.lsc

