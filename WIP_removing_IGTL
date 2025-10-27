Relevant Links:
https://slicer.readthedocs.io/en/latest/developer_guide/script_repository.html 

https://slicer-ros2.readthedocs.io/en/v1.0/pages/nodes/topics.html

https://slicer.readthedocs.io/en/latest/user_guide/getting_started.html#glossary

https://slicer.readthedocs.io/en/latest/user_guide/user_interface.html#mouse-keyboard-shortcuts

https://slicer.readthedocs.io/en/latest/user_guide/user_interface.html#python-console

https://slicer.readthedocs.io/en/latest/user_guide/coordinate_systems.html#coordinate-system-convention-in-slicer

https://slicer.readthedocs.io/en/latest/user_guide/data_loading_and_saving.html #VTK and MRML (so not DICOM I guess) ###########ATTENTION REQUIRED HERE

https://slicer.readthedocs.io/en/latest/user_guide/modules/index.html ###########ATTENTION REQUIRED HERE

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

1. Once we do . install/setup.bash, we can see /IGTL_POINT_OUT and /IGTL_STRING_OUT to Slicer, but not /IGTL_POINT_IN and /IGTL_STRING_IN from Slicer. Why?
Ans: Because of the topic publishers and their data types. Let's consider, first, the structure of the /IGTL_*_OUT messages. 

ros2 topic echo /IGTL_STRING_OUT name: NeedleShapeHeader data: 2025-10-24 09:43:37.811;303;402;zFrame --- 
ros2 topic echo /IGTL_POINT_OUT name: NeedleShapeZ pointdata: - x: 0.0 y: 0.0 z: 200.0 - x: 0.0 y: 0.0 z: 200.00000000167904 - x: -3.093342991866126e-06 y: -1.2461235326727167e-05 z: 200.5000000015142 - '...'

We have here a data stream out on /IGTL_STRING_OUT and /IGTL_POINT_OUT, the first a header message containing timestamp and identifiers, and the second a 3D curve message with a pointdata array of (x,y,z) coordinates describing the reconstructed shape of a needle.

In the example with the DEFAULT MESSAGE from system integration, the NeedleShapeZ sample is a series of point sampled along the needle's length, starting at z = 200 mm and increasing to around 262 mm, while the x and y drift gradually negative, meaning the needle bends smoothly in a particular direction.

Looking at the igtl_node.cpp in ros2_igtl_bridge, which takes in /IGTL_*_OUT, the internal structure is not changed. Note that the /IGTL_*_IN topics cannot be echoed because they ar einput topics consumed by the bridge rather than normal ROS message topics with a known message type being published by a ROS node.
They are dynamically created subscriptions by the bridge to receive messages you want to send to Slicer, not publishers.

So, /IGTL_*_OUT is published by ROS and sent to Slicer. /IGTL_*_IN is subscribed by ROS and received from Slicer. Since we are not publishing anything on /IGTL_*_IN inside ROS, there is no message in the ROS ecosystem to echo, even though the bridge itself forwards network data to Slicer over the OpenIGTLink socket.

On the backend, this->converterManager->AddConverter(string, 10, "IGTL_STRING_IN", "IGTL_STRING_OUT");
this->converterManager->AddConverter(pointarray, 10, "IGTL_POINT_IN", "IGTL_POINT_OUT");

, with each AddConverter call registering a ROS publisher for the _OUT topic, a ROS subscription for the _IN topic, and an OpenIGTLink message type handler that knows how to serialize/deserialize OpenIGTLink packets.

| Direction | ROS Topic         | Role                                        | Seen by ROS2 tools        | Seen by Slicer                     |
| --------- | ----------------- | ------------------------------------------- | ------------------------- | ---------------------------------- |
| OUT       | `/IGTL_POINT_OUT` | Publishes `ros2_igtl_bridge/msg/PointArray` | ✅ yes (`ros2 topic echo`) | ✅ yes                              |
| IN        | `/IGTL_POINT_IN`  | Subscribes for same type                    | 🚫 no (not published)     | ✅ yes (Slicer sends updates to it) |

When we see the needle in Slicer, that is because Slicer is reading the /IGTL_POINT_OUT data, the outgoing message. Slicer's own messages would come bcak into /IGTL_POINT_IN, but those don't appear as ROS publications unless another ROS node publishes them.

In terms of seeing what Slicer sees, topic echo of /IGTL_*_OUT is for the data Slicer receives and uses to render the needle.

To see what is being recieved from Slicer, the igtl_node.cpp would need to be modified to include for example a publisher republishing /IGTL_POINT_IN data once received by the converter, e.g.:

// Inside RIBConverterPointArray::ReceiveIGTLMessage(...)
// after deserializing the IGTL data:
publisher_->publish(ros_msg);  // republish the converted message

Another option is to log it directly to the console in ProcessIGTLMessage.

| Topic              | Purpose                              | Why `ros2 topic echo` fails | What Slicer “sees”   |
| ------------------ | ------------------------------------ | --------------------------- | -------------------- |
| `/IGTL_STRING_OUT` | Outgoing string header to Slicer     | ✅ works (you can echo)      | ✅ Slicer reads it    |
| `/IGTL_POINT_OUT`  | Outgoing needle geometry to Slicer   | ✅ works (you can echo)      | ✅ Slicer reads it    |
| `/IGTL_STRING_IN`  | Incoming string messages from Slicer | 🚫 subscriber only          | ✅ Slicer writes here |
| `/IGTL_POINT_IN`   | Incoming point data from Slicer      | 🚫 subscriber only          | ✅ Slicer writes here |

To verify latency, add metadata, or add new OpenIGTLink message types, these will be done in the converter layer.

2. Let's take a look at what Slicer does with the NeedleShapeZ data once it receives it, specifically how it renders or updates the MRML node. Let's also consider how perhaps in the Markups module one would manually publish such topic information in Slicer.
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Aside on Slicer organization: https://github.com/smart-needle-manual/system_integration/blob/main/Slicer_general_details
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Inside Slicer, all OpenIGTLink communication is managed through the OpenIGTLinkIF module. It creates connector nodes which are network bridges between Slicer's MRML scene and an external device or program like ros2_igtl_bridge.

Each connector can be a server mode which waits for a connection from ROS, or a Client mod connecting out to ROS (bridge).
Each connector can either send (from Slicer externally) MRML nodes, or receive MRML nodes. That mapping correspondings to /IGTL_*_IN and /IGTL_*OUT topics, respectively.

The Markups module (Markups Fiducials*, Curve, or Line) creates MRML nodes like vtkMRMLMarkupsFiducialNode and vtkMRMLMarkupsCurveNode. These can contain lists of 3D points, e.g. /IGTL_POINT_OUT data type.

3. Studying the information flow.
From shape_sensing_needle.py, we get the ShapeSensingNeedleNode, a ROS2 node that reconstructs the current needle shape from sensor data and publishes it as a geometry_msgs/PoseArray on the topic /state/current_shape.

The essential components of that big file are below:

in publish_shape():
pmat, Rmat = self.get_needleshape()     # Nx3 positions and Nx3x3 rotations
header = Header(stamp=..., frame_id='needle')
msg_shape = utilities.poses2msg(pmat, Rmat, header=header)
self.pub_shape.publish(msg_shape)

So that each mesage looks like:
geometry_msgs/msg/PoseArray
  header:
    frame_id: "needle"
  poses:
    - position: {x: ..., y: ..., z: ...}
      orientation: {x: ..., y: ..., z: ..., w: ...}
    - ...

This is exactly the 3D curve along the needle, sampled at some ds spacing.

ROS2 --> OpenIGTLink has to map this PoseArray into an IGTL-compatible format, since Slicer doesn't understand PoseArray directly. It expects IGTL messages like POINT for Markups/Fiducials/Curves, STRING (metadata/headers), TRANSFORM, IMAGE, etc.

Hence, the /IGTL_POINT_OUT topic and /IGTL_STRING_OUT serve as the translation layer between this /state/current_shape and what Slicer visualizes.

From dummy_smart_needle_interface.py, we get a SmartNeedleInterface node which is a bridge sitting between ROS2 and the OpenIGTLink bridge (igtl_node.cpp).

It subscribes to /needle/state/current_shape [geometry_msgs/PoseArray], extracts and reformats that datat into two IGTL-friendly messages:

String message IGTL_STRING_OUT carrying a text header with metadata and a PointArray message IGTL_POINT_OUT carrying the actual 3-D curve points.

It publishes both every second via timer_shape_callback so igtl_node.cpp can forward them out through the OpenIGTLink socket to Slicer.

| Topic              | Message type                      | Purpose                                           | What Slicer uses it for                                                          |
| ------------------ | --------------------------------- | ------------------------------------------------- | -------------------------------------------------------------------------------- |
| `/IGTL_STRING_OUT` | `ros2_igtl_bridge/msg/String`     | Metadata: timestamp, sample count, frame ID, etc. | Optional. Some Slicer modules use it to identify or synchronize multi-part data. |
| `/IGTL_POINT_OUT`  | `ros2_igtl_bridge/msg/PointArray` | 3-D coordinates of the reconstructed needle shape | The actual geometry Slicer turns into a Markups Curve.                           |

The igtl_node.cpp bridge has converters for both, so each gets serialized as an OpenIGTLink message (STRING and POINT respectively) and sent over the socket.

Again, structure of the outgoing messages:

IGTL_STRING_OUT:
ros2_igtl_bridge/msg/String
  name: "NeedleShapeHeader"
  data: "2025-10-24 09:43:37.811;303;402;zFrame"
IGTL_POINT_OUT:
ros2_igtl_bridge/msg/PointArray
  name: "NeedleShapeZ"
  pointdata:
    - {x: 0.0,     y: 0.0,     z: 200.0}
    - {x: -0.0036, y: -0.0159, z: 202.02}
    ...

Inside Slicer's OpenIGTLinkIF, when it receives a message named NeedleShapeZ, it creates or updates a Markups Curve node called "NeedleShapeZ" using the points and optionally reads NeedleShapeHeader to annotate the time/frame.
That is how we get the visible bending needle.

---------------------------------------------------------------------------------------------------------------------------------

So far we have the node that publishes /state/current_shape and the node that listens to it and republishes IGTL messages.

4. If we want to turn /state/current_shape into IGTL_POINT_OUT data, with or without the STRING_OUT info, with in-built Python scripting and without igtl_node.cpp or dummy_smart_needle_interface.py, can we do it? How? 

STRING_OUT is needed for synchronization or custom modules. The POINT message, with a valid .name field, is enough to make Slicer create or update a Markups node with that name. STRING_OUT is not strictly required to visualize geometry like needle shape, fiducials, curves. etc.

| Goal                                  | Simplest path                     | Notes                                     |
| ------------------------------------- | --------------------------------- | ----------------------------------------- |
| Send `/state/current_shape` to Slicer | Keep `/IGTL_POINT_OUT` publisher  | Current system works fine                 |
| Remove STRING_OUT                     | ✅ Safe                            | Only POINT_OUT is needed                  |
| Remove C++ bridge                     | Use `pyigtl` (pure Python client) | Avoids rebuilding ROS–IGTL bridge         |
| Skip OpenIGTLink entirely             | Use `SlicerROS2`                  | Direct ROS2 in Slicer; no IGTL, pure ROS2 |

Everything up to ShapeSensingNeedleNode contains the true data source, the PoseArray describing needle shape. Everything after that is just packaging and transport.

The real goal is to let Slicer directly see /state/current_shape without any intermediate 'bridge' or redundant code. We will use Slicer-native tools.

Independent, built-in Slicer options:

| Connection type      | Uses               | Protocol                 | Best for                                       |
| -------------------- | ------------------ | ------------------------ | ---------------------------------------------- |
| 🧩 **OpenIGTLinkIF** | “legacy” method    | OpenIGTLink TCP messages | Hardware devices, pre-built bridges            |
| 🧠 **SlicerROS2**    | direct ROS2 client | DDS (ROS2 middleware)    | Native ROS2 topics like `/state/current_shape` |

The second option makes sense for our use case. We will use a SlicerROS2 extension. It speaks to ROS2 natively, without wrappers, IGTL_POINT_OUT, or metadata strings.

Problem assumptions:
- Slicer build has SlicerROS2 extension installed.
- ROS2 network is already working, e.g. ros2 topic list shows /state/current_shape.

Inside Slicer's Python interactor or in a scripted module, we can do:

import slicer
import rclpy
from geometry_msgs.msg import PoseArray
import vtk

# 1️⃣ Initialize ROS2 (only once)
rclpy.init()
node = rclpy.create_node('slicer_needle_listener')

# 2️⃣ Callback to convert ROS PoseArray → Slicer Markups curve
def shape_callback(msg):
    # Clear old data (optional: reuse a node)
    name = "NeedleShape"
    existing = slicer.util.getNode(name) if slicer.util.getNodesByClassByName('vtkMRMLMarkupsCurveNode', name) else None
    curveNode = existing or slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsCurveNode", name)
    curveNode.RemoveAllControlPoints()
    
    # Add points from PoseArray
    for pose in msg.poses:
        curveNode.AddControlPointWorld(vtk.vtkVector3d(pose.position.x, pose.position.y, pose.position.z))

    # Optional: auto-display or color
    displayNode = curveNode.GetDisplayNode()
    if displayNode:
        displayNode.SetColor(1, 0, 0)  # Red line
        displayNode.SetLineThickness(2.0)

# 3️⃣ Subscribe to your topic
sub = node.create_subscription(PoseArray, '/state/current_shape', shape_callback, 10)

# 4️⃣ Run ROS spin loop in background
import threading
def ros_spin():
    rclpy.spin(node)
spin_thread = threading.Thread(target=ros_spin, daemon=True)
spin_thread.start()

------------------------------------------------------------------------------------------------------------------------------

This script subscribes to /state/current_shape from ShapeSensingNeedleNode, creates a Markups Curve node in the Slicer scene, updates it live as the PoseArray changes, and does not use ros2_igtl_bridge, C++ nodes, or OpenIGTLink.

Slicer ←→ ROS2 over DDS, one script, all in Python.

Why this works:

/state/current_shape contains geometry_msgs/PoseArray, ideal for describing a path or 3D curve; Slicer's Python API speaks VTK coordinate arrays, which map 1:1 with ROS positions; and ~20 lines are needed to logically go from one to the other.

No serialization, intermediate topics, or ROS bridge.

Option to make the code persistent or modular: either wrap the code in a Slicer scripted module so it autostarts when Slicer opens, or use Slicer's Python startup script 

.slicerrc.py 

to auto-connect at launch.

E.g.:

# ~/.slicerrc.py
import slicer
import my_slicer_ros2_bridge  # Your script from above
my_slicer_ros2_bridge.start()
------------------------------------------------------------------------------------------------------------------------------
| Method                           | Code Required | Bridge      | Real-time | Notes                |
| -------------------------------- | ------------- | ----------- | --------- | -------------------- |
| `igtl_node.cpp` + OpenIGTLinkIF  | Heavy         | C++         | ✅         | Legacy standard      |
| `pyigtl` inside ROS              | Medium        | Python-only | ✅         | Good fallback        |
| **SlicerROS2 Python subscriber** | 💡 Minimal    | ❌ None      | ✅         | Most direct & modern |

Given the existing stack, keep ShapeSensingNeedleNode exactly as it is, remove smart_needle_interface and igtl_node.cpp, and add the SlicerROS2 Python subscriber.

This is the cleanest and lightest architecture:
ShapeSensingNeedleNode
      ↓
  /state/current_shape  (PoseArray)
      ↓
  SlicerROS2 Python script
      ↓
  vtkMRMLMarkupsCurveNode ("NeedleShape")

#Example which builds a self-contained Slicer Python module. This subscribes to /state/current_shape ROS2 topic, converts the PoseArray into a Markups Curve, and updates it live in the Slicer scene.
"""
NeedleShapeSubscriber.py
Slicer Python module to subscribe to ROS2 PoseArray and display as a Markups Curve
"""

import slicer
import rclpy
from geometry_msgs.msg import PoseArray
import vtk
import threading

class NeedleShapeSubscriber:
    def __init__(self, topic_name='/state/current_shape', curve_name='NeedleShape'):
        self.topic_name = topic_name
        self.curve_name = curve_name

        # 1️⃣ Initialize ROS2 if not already initialized
        try:
            rclpy.get_default_context()
        except RuntimeError:
            rclpy.init()

        self.node = rclpy.create_node('slicer_needle_listener', namespace='')

        # 2️⃣ Subscribe to PoseArray
        self.subscriber = self.node.create_subscription(
            PoseArray,
            self.topic_name,
            self.shape_callback,
            10
        )

        # 3️⃣ Start ROS spin in a background thread
        self.spin_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.spin_thread.start()

        # 4️⃣ Create the curve node (or get existing)
        existing_nodes = slicer.util.getNodesByClassByName('vtkMRMLMarkupsCurveNode', self.curve_name)
        if existing_nodes:
            self.curve_node = existing_nodes[0]
        else:
            self.curve_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsCurveNode", self.curve_name)
            display_node = self.curve_node.GetDisplayNode()
            if display_node:
                display_node.SetColor(1, 0, 0)  # Red line
                display_node.SetLineThickness(2.0)

        slicer.app.processEvents()

    def ros_spin(self):
        """Run ROS2 event loop in background"""
        rclpy.spin(self.node)

    def shape_callback(self, msg):
        """Convert PoseArray to Markups Curve"""
        if not self.curve_node:
            return

        # Remove old points
        self.curve_node.RemoveAllControlPoints()

        # Add points from PoseArray
        for pose in msg.poses:
            self.curve_node.AddControlPointWorld(vtk.vtkVector3d(
                pose.position.x,
                pose.position.y,
                pose.position.z
            ))

        slicer.app.processEvents()  # Refresh Slicer scene

    def shutdown(self):
        """Stop ROS2 node cleanly"""
        self.node.destroy_node()
        rclpy.shutdown()
        self.spin_thread.join()

# ----------------------
# Example usage in Slicer Python Interactor
# ----------------------
# subscriber = NeedleShapeSubscriber()
# (The curve will update live)
# To stop: subscriber.shutdown()
--------------------------------------------------------------------------------------------------------------------------

Features include single python file, run from Python Interactor or dropped into Slicer's Python module directory. Direct ROS2 subscription without IGTL bridge. Updates Markups Curve whenever a new PoseArray arrives. CUstomizable to change topic_name or curve_name for multiple needles. Safe shutdown using shutdown() when want to stop ROS cleanly.

#Example run:

# Launch subscriber
subscriber = NeedleShapeSubscriber(topic_name='/state/current_shape', curve_name='NeedleShape')

# Later, if you need to clean up
subscriber.shutdown()
-----------------------------------------------------------------------------------------------------------------------------
#To update this to also visualize tip orientation as is done in the SmartNeedleInterface, enhance Slicer-side script so it draws curve from /state/current_shape, extracts tip pose fom last Pose in PoseArray, and visualizes it as a small coordinate frame (axes) that updates in real time.

"""
NeedleShapeSubscriberWithTip.py
Subscribe to ROS2 PoseArray (/state/current_shape)
Display shape as a Markups Curve and visualize tip orientation in 3D
"""

import slicer
import rclpy
from geometry_msgs.msg import PoseArray
import vtk
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R

class NeedleShapeSubscriberWithTip:
    def __init__(self, topic_name='/state/current_shape', curve_name='NeedleShape', tip_name='NeedleTip'):
        self.topic_name = topic_name
        self.curve_name = curve_name
        self.tip_name = tip_name

        # 1️⃣ Initialize ROS2 (if not already)
        try:
            rclpy.get_default_context()
        except RuntimeError:
            rclpy.init()

        self.node = rclpy.create_node('slicer_needle_listener')

        # 2️⃣ Subscribe to PoseArray
        self.subscriber = self.node.create_subscription(
            PoseArray,
            self.topic_name,
            self.shape_callback,
            10
        )

        # 3️⃣ Start ROS2 spin loop
        self.spin_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.spin_thread.start()

        # 4️⃣ Create or reuse the curve
        existing_curves = slicer.util.getNodesByClassByName('vtkMRMLMarkupsCurveNode', self.curve_name)
        self.curve_node = existing_curves[0] if existing_curves else slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsCurveNode", self.curve_name)
        self.curve_node.GetDisplayNode().SetColor(1, 0, 0)
        self.curve_node.GetDisplayNode().SetLineThickness(2.0)

        # 5️⃣ Create or reuse a tip coordinate frame model
        existing_tips = slicer.util.getNodesByClassByName('vtkMRMLModelNode', self.tip_name)
        self.tip_node = existing_tips[0] if existing_tips else slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", self.tip_name)
        self.tip_node.CreateDefaultDisplayNodes()
        self.tip_node.GetDisplayNode().SetColor(0, 1, 0)
        self.tip_node.GetDisplayNode().SetSliceIntersectionVisibility(True)
        self.tip_node.GetDisplayNode().SetVisibility3D(True)

        # Create coordinate axes model for the tip
        axes_source = vtk.vtkAxesActor()
        axes_source.SetTotalLength(10.0, 10.0, 10.0)
        self.axes_transform = vtk.vtkTransform()
        self.tip_node.SetAndObserveTransformNodeID(None)
        self.tip_node.SetUserMatrix(self.axes_transform.GetMatrix())

        slicer.app.processEvents()

    def ros_spin(self):
        """Background ROS2 spin"""
        rclpy.spin(self.node)

    def shape_callback(self, msg):
        """Called every time a new PoseArray arrives"""
        if not msg.poses:
            return

        # --- Update the curve ---
        self.curve_node.RemoveAllControlPoints()
        for pose in msg.poses:
            self.curve_node.AddControlPointWorld(vtk.vtkVector3d(
                pose.position.x,
                pose.position.y,
                pose.position.z
            ))

        # --- Extract the tip pose (last one) ---
        tip_pose = msg.poses[-1]
        tip_position = np.array([tip_pose.position.x, tip_pose.position.y, tip_pose.position.z])
        q = np.array([tip_pose.orientation.x, tip_pose.orientation.y, tip_pose.orientation.z, tip_pose.orientation.w])

        # Convert quaternion → rotation matrix
        Rmat = R.from_quat(q).as_matrix()
        T = np.eye(4)
        T[:3, :3] = Rmat
        T[:3, 3] = tip_position

        # Apply transform to tip visualization
        m = vtk.vtkMatrix4x4()
        for i in range(4):
            for j in range(4):
                m.SetElement(i, j, T[i, j])
        self.tip_node.SetAndObserveMatrixTransformToParent(m)

        slicer.app.processEvents()

    def shutdown(self):
        """Gracefully close the ROS node"""
        try:
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        if self.spin_thread.is_alive():
            self.spin_thread.join()


# ----------------------------
# Example usage inside Slicer:
# ----------------------------
# subscriber = NeedleShapeSubscriberWithTip('/state/current_shape')
# To stop later:
# subscriber.shutdown()
-----------------------------------------------------------------------------------------------------------------------------
What this does:
- Subscribes to /state/current_shape directly via ROS2
- Draws red curve of needle shape
- Creates a green 3D axes marker at the tip pose
- Updats both live as data streams in
- No igtl_node.cpp, no smart_needle_interface, no bridge. Just Slicer + ROS2.

Notes: Requires Slicer with ROS2 Python bindings, e.g. running in a ROS2 environment. If you get a "Cannot import geometry_msgs" error, you need to launch Slicer inside your ROS2 workspace:

source /opt/ros/humble/setup.bash
./Slicer

Adjust tip axes via:

axes_source.SetTotalLength(5.0, 5.0, 5.0)








