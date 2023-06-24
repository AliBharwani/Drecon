# DReCon Unity Implementation

![GIF](gifs/drecon-short-gif.gif)
Youtube video: https://youtu.be/dqIwWVMw7HM

This is a reimplementation of the [Drecon paper](https://www.theorangeduck.com/media/uploads/other_stuff/DReCon.pdf) [[1]](#citation1) for user controlled physically-simulated characters using Unity-ML Agents.

## Installation / Running

In order to run this scene, you can download the [executable from here](https://alibharwani.itch.io/drecon-demo-unity) or load it in the Unity editor (2021.3.9f).

Most of the relevant settings for demo-ing are under "Inference Settings" on the `Singletons > ConfigManager` object. 

In order to control the simulated character with a gamepad or keyboard, ensure "User Control" is ticked and start the scene (or disable `gen_inputs` on `MotionMatchingAnimator.cs` after starting). 

Once the scene is playing, you can also enable the "CameraWASDController". This controller will let you move around the environment freely and launch projectiles by left clicking. `gen_inputs` must be true on `MotionMatchingAnimator.cs` in order to use this controller - it can be set true manually in the editor or by having "User Control" on the ConfigManager object set to false when starting the scene.


## Overview 
I adapted Daniel Holden's [implementation of motion matching](https://github.com/orangeduck/Motion-Matching)  [[2]](#citation2) for the kinematic character. The kinematic character is the white animated character that the simulated character is trying to match. The neural net learns to produce rotation offsets for the PD motors that represent its muscles, with the starting rotations supplied by the kinematic character every frame. 

Motion matching is an animation technique that queries a large motion database multiple times a second to retrieve the most relevant clip to blend to based on user input and the current pose. Most of the logic here is the same as in Holden's, though I used my own implementation of a KDTree which is used to speed up the query. 

The database file that is read from is also the same as the one produced by the `generate_database` file from  [[2]](#citation2), though the quaternions are multiplied by (1, -1, -1, 1) to work for Unity.
 
Most of the logic lives in `MLAgent.cs`

The script execution order is set in the Player Settings. The anatomy of a frame:

(1) `MotionMatchingAnimator.cs` updates the kinematic character
(2) `MLAgent.cs` updates observation state variables that are dependent on the kinematic character. It either requests an action or applies the actions from the last network output. 
(3) `AcademyManager.cs` runs one step. If `MLAgent.cs` requested a decision, it will execute the `CollectObservations()` and `OnActionReceived()`  functions of `MLAgent.cs`
(4) `PhysicsManger.cs` steps the simulation by the fixed timestep. This has to run after the PD motor targets are updated by the network in (3) or (4) 
(5) `MultiMLAgentsDirector.cs` executes. This calls the `LateFixedUpdate()` method on `MLAgent.cs`, which calculates the rewards and updates data dependent on the simulated model (center of mass, etc) that changed after the physics step. 

### ConfigManager
The ConfigManager object controls all of the relevant settings for the motion matching system, the simulated character, and the training run. It also has utility functions for writing and loading to JSON. Depending on the combination of settings, the number of inputs or outputs to the network can change, and `MultiMLAgentsDirector` will instantiate the proper `MLAgent` when the scene is started. 

### Differences from the DreCon paper
I experimented a lot with various variables, but ultimately had similar ablation results to the ones found in the original paper. The only differences worth noting are: 
1) Setting the drive target velocities as well (they also mention doing this in the SuperTrack [[3]](#citation3) paper) 
2) Using a custom input generator and transitioning between generated inputs to get a more realistic simulation of user inputs
3) Using a different rig for the physical character. I initially used a dynamically created body [[4]](#citation4), but human bodies are asymmetrical and uneven. I decided to use a manual rig to train on instead, though some parts (rotation limits) were still dynamically generated. 

I also tested several different representations of the rotation outputs, but found that Euler works best. Since most of the joints are limited in their range, the usual shortcomings of Euler representations (gimbal lock, discontinuity) are obviated.
## Training 

Once you have your ConfigManager setup with the settings you like, you can use the `mlagents-learn` command to train.

By default, `mlagents` does not accept a "init_near_zero" argument in the config. In order to use this feature, you must manually alter your installation of the package. It also restricts the number of available CPUs to 4, and this is a very heavily CPU bottlenecked training, so it is suggested to modify that as well.

The `mlagents-learn` command will, by default, run the simulation at 20x speed. This makes the simulation very unstable, and it's more efficient to run many parallel instances rather than try to run one simulation at that speed. In order to turn this off, be sure to include the args `--time-scale 1 --capture-frame-rate 0`

What I found was the most stable, efficient running setup was to have 2 agents in each build (so make sure the MultiMLAgentsDirector has '2' set for "Num Agents") and run 10-15 no graphics instances, depending on your CPU speed and GPU memory. My final training command would look something like: 
`mlagents-learn Assets\Config\Drecon.yaml --env="Builds\Drecon.exe" --no-graphics --num-env 10 --time-scale 1 --capture-frame-rate 0 --run {run_id}`

Be sure to right click on the ConfigManager in the editor and "Write out current config to config name" in order to save a usable unityconfig to the folder for your training run.

## Future Work

Currently, I'm focusing on re-implementing SuperTrack 1.  [[3]](#citation3), so I do not expect to update this repo in the future. However, there is a lot that can be done with the existing code here. 

Since most of the code is data driven, we can quite easily expand the animation system to animate different types of movement (different gaits/body shapes/styles) and train the simulated character on the new movement. The only requirement would be having sufficient (5-10 min, preferably) motion capture data. The database can be generated by the file in [[2]](#citation2), the character can be imported as an FBX and have its colliders and articulation bodies dynamically created [[4]](#citation4), and the network can be trained to follow this new type of movement. 

One of the clear weaknesses of this model is its inability to stand still. There is at trade-off between the character's ability to make rapid movements and its ability to move smoothly and twitchiness. It would be interesting to experiment will allowing the model to output its own damping parameter(s), allow the policy to determine when it should fire next, or altering the setup of the articulation bodies in order to overcome this.

<a id="citation1"></a>[1]  https://theorangeduck.com/media/uploads/other_stuff/DReCon.pdf

<a id="citation2"></a>[2] https://github.com/orangeduck/Motion-Matching

<a id="citation3"></a>[3] https://theorangeduck.com/media/uploads/other_stuff/SuperTrack.pdf

<a id="citation4"></a>[4] https://alibharwani.com/2022/09/29/bounding-capsules/