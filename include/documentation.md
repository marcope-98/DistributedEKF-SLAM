
# Classes Documentation
Table of contents:
- [Actuator](#Actuator) 
- [Agent](#Agent) 
- [ekfSLAM](#ekfSLAM)
- [Message](#Message)
- [Sensor](#Sensor)
- [Server](#Server)



## Actuator

The `Actuator` class stores the odometry data of the MRCLAM Dataset.
The Odometry data is formatted in the following way:

<div align="center">

|  | Time [s] | forward velocity [m/s] | angular velocity [rad/s] |
|   :---:  |   :---:  |         :---:          |          :---:           |
| current | ... | ... | ... |

</div>

**Properties**

- `current` : pointer to the current measurement row to read from
- `Odometry`: complete odometry data
- `dt`      : simulation timestep


**Methods**

- `Actuator(sim_start, Odometry)`

    Constructor of the class. 
    - `sim_start` simulation start index
    - `Odometry`  full odometry data
- `[t, u] = control()`

    The control method fetches the odometry information relative to the current time step of the simulation.

    The method returns the current time instant `t` (first column of the odometry data) and the controls `u`.
    
    The control `u` is a 3x1 vector of the form `[v; w; dt]` where:
    - `v` is the linear  velocity
    - `w` is the angular velocity
    - `dt` is the timestep.


## Agent

The `Agent` class is the main protagonist of  the project. It makes use of all of the classes documented in this repository. Its most important functionalities are performing a step of the EKF SLAM algorithm, broadcasting its current state and covariance matrix to other agents and estimating the position of the landmarks based on the information received from other agents.


**Properties**
- `ekf` ekfSLAM object responsible for performing the prediction and correction steps
- `sensor` Sensor object responsible to retrieve the measurement sensed at a specific time step
- `actuator` Actuator object responsible to retrieve the control information
- `server` server object responsible from dispatching messages between robots
- `id` unique identifier representing the agent
- `sim` Simulation details such as number of robots, start and end of the simulation
- `info` Ground truth pose of the robot (used ONLY for initialization)
- `Est` pose history as estimated using the information consensus and the EKF SLAM algorithm

**Methods**

`Agent(Robot, codeDict, server, params)`

    Constructor of the class that construct all the necessary objects to perform Cooperative SLAM


`step()`

    The `step` function retrieves the current control inputs, the sensed landmarks and surrounding agents, performs the prediction and correction step of the EKF SLAM algorithm and broadcast to the sensed robots the state and covariance matrix of the landmarks.

`broadcast(receipients)`

    The `broadcast` function generate the message to be sent and communicates to the server such message specifying sender (`id`) and the receipient (which is contained in receipients).

`fetch()`

    `fetch` method retrieves the messages received from other robots from the server and estimates the position of the landmarks based on the information consensus.

In particualr the `fetch` function assumes that each landmark state is statistically indipendent with uncertainty represented by a Gaussian.
The resulting landmark state is the MVUE estimate of the information received from other robots and the current estimate of the receiving agent.

This can be easily implemented by inverting each 2x2 covariance matrix representing the `x` and `y` coordinates of the landmark obtaining the Fisher Information Matrix. Similarly one can obtain the information vector as follows:

$$
Y = \Sigma^{-1} \qquad\qquad y = Y x
$$

where
- $Y$ is the information matrix
- $\Sigma$ is the covariance matrix (2x2) of the landmark coordinates
- $x$ is the state of the landmark (`x` and `y` estimated coordinates)
- $y$ is the information vector

Given the additive property of the Fisher Information Matrix one can perform separates information consensus operations (one for each landmark) given the information corresponding to the same landmark received from other robots.

In particular the MVUE estimate of a single landmark is obtained as follows:

$$
y_{MVUE} = \frac{1}{R}\sum_{i=1}^R y_i \qquad\qquad Y_{MVUE} = \frac{1}{R}\sum_{i=1}^R Y_i
$$

where $R$ is the number of messages received.

The estimated state from the consensus is obtained by inverting the equation of the Fisher Information Matrix, i.e.

$$
\Sigma = Y^{-1} \qquad\qquad x = Y^{-1} y
$$

`cvt_to_Robot()`

    Helper method to convert the Agent state and member variables in a suitable format for visualization


`state = get_state()`

    Helper method to retrieve the current pose of the Agent.

`landmarks = get_landmarks()`

    Helper method to retrieve the `x` and `y` coordinates of the landmarks.

`Agent(Robot, codeDict, server, params)`

    Constructor of the class that construct all the necessary objects to perform Cooperative SLAM


`step()`

    The `step` function retrieves the current control inputs, the sensed landmarks and surrounding agents, performs the prediction and correction step of the EKF SLAM algorithm and broadcast to the sensed robots the state and covariance matrix of the landmarks.

`broadcast(receipients)`

    The `broadcast` function generate the message to be sent and communicates to the server such message specifying sender (`id`) and the receipient (which is contained in receipients).

`fetch()`

    `fetch` method retrieves the messages received from other robots from the server and estimates the position of the landmarks based on the information consensus.

In particualr the `fetch` function assumes that each landmark state is statistically indipendent with uncertainty represented by a Gaussian.
The resulting landmark state is the MVUE estimate of the information received from other robots and the current estimate of the receiving agent.

This can be easily implemented by inverting each 2x2 covariance matrix representing the `x` and `y` coordinates of the landmark obtaining the Fisher Information Matrix. Similarly one can obtain the information vector as follows:

$$
Y = \Sigma^{-1} \qquad\qquad y = Y x
$$
where
- $Y$ is the information matrix
- $\Sigma$ is the covariance matrix (2x2) of the landmark coordinates
- $x$ is the state of the landmark (`x` and `y` estimated coordinates)
- $y$ is the information vector

Given the additive property of the Fisher Information Matrix one can perform separates information consensus operations (one for each landmark) given the information corresponding to the same landmark received from other robots.

In particular the MVUE estimate of a single landmark is obtained as follows:

$$
y_{MVUE} = \frac{1}{R}\sum_{i=1}^R y_i \qquad\qquad Y_{MVUE} = \frac{1}{R}\sum_{i=1}^R Y_i
$$
where $R$ is the number of messages received.

The estimated state from the consensus is obtained by inverting the equation of the Fisher Information Matrix, i.e.

$$
\Sigma = Y^{-1} \qquad\qquad x = Y^{-1} y
$$

`cvt_to_Robot()`

    Helper method to convert the Agent state and member variables in a suitable format for visualization


`state = get_state()`

    Helper method to retrieve the current pose of the Agent.

`landmarks = get_landmarks()`

    Helper method to retrieve the `x` and `y` coordinates of the landmarks.

## ekfSLAM
The `ekfSLAM` class implements all the elementary operation to perform the Extended Kalman Filter SLAM (with known data association) algorithm.

The class makes the following assumptions:
1. The agents operate in the plane (i.e. the state of the robot is expressed as `x`, `y` and `theta`)
2. The measurements are expressed using a range and bearing observation model
3. The controls are expressed using a velocity based motion model.
4. The number of landmarks is known a priori

The EKF SLAM keeps an augmented vector ($\mu$) and its corresponding covariance matrix $\Sigma$.

In particular, the state vector takes the following form:

$$
\mu = [x, y, theta, x_1, y_1, ... , x_N, y_N]^T
$$

where the first three components are the estimated robot pose and the remaining elements are the estimated `x` and `y` coordinates of the `N` landmarks.

Similarly the covariance matrix takes the form:

$$
\Sigma =
\begin{bmatrix} 
\Sigma_{xx} & \Sigma_{mx} \\
\Sigma_{xm} & \Sigma_{mm}
\end{bmatrix} $$

where:
- $\Sigma_{xx}$ is the covariance matrix of the robot pose (representing the uncertainty in the estimated robot pose)
- $\Sigma_{xm}$ is the covariance matrix of the robot pose and local map
- $\Sigma_{mm}$ is the covariance matrix of the map (landmark `x` and `y` coordinates)

The EKF SLAM with known correspondence involves a prediction step and a correction step.
The prediction step uses the control vector to estimate the next state (`x`, `y` and `theta`) of the robot as well as the covariance matrix involving these three quantities.
It can be summarized as follow:


$$
\begin{align*}
\bar{\mu_t}    &= g(u_t, \mu_{t-1})\\
\bar{\Sigma_t} &= G_t \Sigma_{t-1} G_t^T + R_t
\end{align*}
$$


where:
- $\bar{\mu}_t$ is the predicted state
- $g(u_t, \mu_{t-1})$ is the motion model
- $\bar{\Sigma}_t$ is the predicted covariance matrix
- $G_t$ is the jacobian of the motion model wrt the robot pose (`x`, `y` and `theta`)
- $R_t$ is the process noise matrix (`v` and `w`)

The correction step makes use of the measurement from the range sensor to correct the predicted pose of the robot. It can be summarized as follows:

$$
\begin{align*}
S_t &= H_t\bar{\Sigma}_tH_t^T + Q_t \\
K_t &= \bar{\Sigma}_t\, H_t^T \, S_t^{-1} \\
\mu_t &= \bar{\mu}_t + K_t(z_t - h(\bar{\mu}_t)) \\
\Sigma_t &= (I - K_t H_t) \bar{\Sigma}_t
\end{align*}
$$


where:
- $S_t$ is the innovation covariance
- $K_t$ is the Kalman Gain
- $z_t$ is the observed measurement (range and bearing)
- $h(\cdot)$ is the observation model
- $H_t$ is the jacobian of the observation model wrt the robot pose.

**Properties**

- `Q` is the measurement noise matrix
- `A` is a set of parameters used to compute the process noise matrix
- `N` is the number of landmarks
- `state` is the augmented vector ($\mu$)
- `cov` is the covariance matrix ($\Sigma$)
- `observed` is a vector of boolean values that keeps track of which landmarks have already been observed.


**Methods**

`ekfSLAM(state_0, params)`

    Constructor of the class.
- `state_0` contains the ground truth pose of the robot at the start of the simulation
- `params` is a structure containing the number of landmark (`params.N`), the parameters $\alpha$ in a row vector (`params.A`) and the measurement noise matrix (`params.Q`)


`predict(u)`

    EKF SLAM prediction step.
During the prediction step, the method applies the motion model $g(\cdot)$ and retrieves its Jacobian wrt the robot pose (`x`, `y` and `theta`) and control input (`v` and `w`).

In particular, the member variable `A` in the form
$$A = \left[ \alpha_1 , \alpha_2 , \alpha_3 , \alpha_4 \right]$$
is used to compute the process noise matrix as follows:

$$
U = 
\begin{bmatrix} 
(\alpha_{1} |v| + \alpha_{2} |\omega|)^2 & 0 \\
0 & (\alpha_{3} |v| + \alpha_{4} |\omega|)^2
\end{bmatrix} $$

`[g, G, V] = velocity_motion_model(u)`

    The motion model of choice for this implementation of the EKF SLAM algorithm is a velocity based motion model, which takes the general form:

$$
g = \begin{bmatrix} - \frac{v}{\omega} (\sin(\theta) - \sin(\theta + \omega \cdot\delta t))\\
+\frac{v}{\omega} (\cos(\theta) - \cos(\theta + \omega\cdot\delta t))\\
\omega \cdot \delta t \end{bmatrix}
$$

In addition to these quantities, the velocity motion model also computes the jacobian of the motion model wrt the robot pose (`x`, `y` and `theta`) and the control inputs (`v` and `w`):

$$
G = \begin{bmatrix} 1 & 0 & -\frac{v}{\omega} (\cos(\theta) - \cos(\theta + \omega\cdot\delta t))\\
0 & 1 & - \frac{v}{\omega} (\sin(\theta) - \sin(\theta + \omega \cdot\delta t))\\
0 & 0 & 1\end{bmatrix}\\
$$

$$
V = \begin{bmatrix}-\frac{v}{\omega^2} (\sin(\theta) - \sin(\theta + \omega \cdot\delta t)) & +\frac{v}{\omega^2} (\sin(\theta) - \sin(\theta + \omega \cdot\delta t)) + \frac{v}{\omega} \cos(\theta + \omega \cdot \delta t)\delta t\\
+\frac{v}{\omega^2} (\cos(\theta) - \cos(\theta + \omega\cdot\delta t)) & -\frac{v}{\omega^2} (\cos(\theta) - \cos(\theta + \omega \cdot\delta t)) + \frac{v}{\omega} \sin(\theta + \omega \cdot \delta t)\delta t\\
0 & \delta t\end{bmatrix}
$$

Whenever the singularity $\omega \to 0$ is reached another set of equations is used:

$$
g = 
\begin{bmatrix}
    v \cdot \delta t \cos(\theta)\\
    v \cdot \delta t \sin(\theta)\\
    0
\end{bmatrix}
$$

$$
G = 
\begin{bmatrix}
1 & 0 & -v \cdot \delta t \sin(\theta) \\
0 & 1 & +v \cdot \delta t \cos(\theta) \\
0 & 0 & 1
\end{bmatrix}
$$

$$
V = 
\begin{bmatrix}
\cos(\theta) \delta t& -v \sin(\theta) \frac{\delta t^2}{2}\\
\sin(\theta) \delta t& +v \cos(\theta) \frac{\delta t^2}{2}\\
0&\delta t
\end{bmatrix}
$$

`zHat = h(landmark)`

    observation model and computes the range and bearing quantities from the estimate of the landmark position using the predicted state as the origin.

Given 
$$\delta x = landmark_x - \mu_x\qquad\qquad \delta y = landmark_y - \mu_y$$
The observation model outputs the expected range and bearing as follows:

$$
\hat{z} = 
\begin{bmatrix}
\sqrt{\delta x^2 + \delta y^2} \\ 
atan2(\delta y, \delta x) - \mu_{\theta}
\end{bmatrix}
$$

`H3 = dh_dstate(landmark)`

    `dh_dstate` method computes the jacobian of the measurement model wrt the robot pose.
    
Given
$$\delta x = landmark_x - \mu_x\qquad\qquad \delta y = landmark_y - \mu_y$$
and 
$$q = \delta x^2 + \delta y^2$$
The Jacobian takes the form:

$$
H = \begin{bmatrix}
    -\frac{\delta x}{\sqrt{q}} & -\frac{\delta y}{\sqrt{q}} & 0\\
    +\frac{\delta y}{q} & -\frac{\delta x}{q} & -1 
\end{bmatrix}
$$

`correct(measurement)`

    The `correct` methods applies the correction step of the EKF SLAM algorithm.

`pose = get_pose()`

    Getter method to retrieve the estimated pose of the robot.

`landmarks = get_landmarks()`

    Getter method to retrieve the `x` and `y` coordinates of the landmarks



## Message
The `Message` class stores the content that one wish to communicate to other robots/agents. In other terms it represents a common interface for the information to be passed around.

**Properties**

- `content` : structure mimicking the content of the message (empty by default)

**Methods**

- `Message(varargin)`

    Constructor of the Message. It accepts a variable number of inputs:
    - if `nargin > 0` the first argument passed to the constructor is stored in the `content` variable
    - otherwise the constructor initializes an empty message

- `flag = isempty()`

    The methods checks whether or not the message is empty. This is done by checking if the structure `content` is empty.


## Sensor
The `Sensor` class stores the measurement data of the MRCLAM Dataset. The Measurement data is formatted in the following way:

<div align="center">

|  | Time [s] | Barcode # | range [m] | bearing [rad] |
|:---:|:---:|:---:|:---:|:---:|
| current | ... | ... | ... | ... |
</div>

The dataset comprises a total of 20 barcodes: the first 5 are reserved for the robots, the remaining refers to landmarks.

The Barcode is a unique identifies associated with each component of the environment that is uniquely mapped to a specific value. The map between barcodes and agent/landmark is also provided by the dataset


**Properties**

- `current` : pointer to the current measurement row to read from
- `Measurement`: complete measurement data
- `codeDict`      : HashMap that maps Barcodes to Landmark IDs

**Methods**

- `Sensor(codeDict, Measurement)`

    Constructor of the class.
    - `CodeDict` is the hashmap for the association of barcodes to landmarks
    - `Measurement` is the complete measurement information provided by the dataset

- `varargout = sense(t)`

    The sense method fetches all the measurement from the `current` index until the the time step `t`, since multiple landmarks and robots can be seen at a time instant.
    
    The method return a variable number of output arguments
    - The first output argument is a list of landmark measurements. Each row represents a different measurement in the form
    
        `[range; bearing; landmarkID]`.
    - The second output argument is a list of robot IDs within the sensing range.


## Server
The `Server` class is an helper class used to mimick p2p communication within robots. The Server object is used to store and distribute the message from each robot/agent to its receipients. In this way, even though the simulation is performed sequentially, it is as if inter process communication and synchronization within the robots/agents takes place.

**Properties**

- `N` number of agents/nodes in the network
- `inbox` cell array containing the message to be received by each agent.

NOTE: Assuming `N` agents in the network, the `inbox` member variable, contains `N` array of `N` messages (either empty or not). It can be interpreted as a `N`x`N` matrix where each row represents the receiver and each column the sender of the message.

**Methods**

- `Server(nRobots)`

    Constructor of the class.
    - `nRobots` is the number of agents/nodes in the network

- `send(from, to, message)`

    The `send` method represents the interface by which each robot can communicate with the `Server` object.
    - `from` is the id of the sender
    - `to` is the id of the receiver
    - `message` is the message to be communcated

    NOTE: in case `to` is higher than the number of robots in the network, the message is ignored.

- `flush()`

    The `flush` method reset the state of the `inbox` filling it with empty messages.
    This method HAS to be called either at the beginning of each iteration step or at the end.

