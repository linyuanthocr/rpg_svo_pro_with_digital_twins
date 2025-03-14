# Steps to Enable GUI in Your Existing Docker Container

## 1️⃣ On Your Host Machine (Outside Docker)

Allow local GUI access for your container:

```bash
xhost local:root
```

(This is more secure than `xhost +`.)

## 2️⃣ Enter Your Existing Docker Container

Find your running container's name or ID:

```bash
docker ps
```

It should show something like:

```
CONTAINER ID   IMAGE                          COMMAND   CREATED         STATUS        NAMES
abcd1234       osrf/ros:noetic-desktop-full   "bash"    30 minutes ago   Up 30 min   ros_noetic
```

Now, enter the container:

```bash
docker exec -it ros_noetic bash
```

(Replace `ros_noetic` with your actual container name if it's different.)

## 3️⃣ Set Up the Display Variable Inside Docker

Once inside the container, run:

```bash
export DISPLAY=$DISPLAY
```

Then verify with:

```bash
echo $DISPLAY
```

If it returns an empty value, manually set it to `:0`:

```bash
export DISPLAY=:0
```

To make the setting persistent, run:

```bash
echo 'export DISPLAY=:0' >> ~/.bashrc
source ~/.bashrc
```

## 4️⃣ Run RViz

Now, try running:

```bash
rviz
```

RViz should now **open on your host machine**.
