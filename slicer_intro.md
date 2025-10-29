## 🧭 3D Slicer Core Concepts (Reorganized Overview for Python Scripting)

### 1. Orientation and the Slicer Scene

Everything in Slicer revolves around the **MRML Scene** — the in-memory data repository that stores and organizes all loaded datasets (volumes, models, markups, transforms, etc.).

* **Scene = `vtkMRMLScene`**

  * Contains a collection of **nodes** (each node represents a dataset or display/configuration object).
  * Nodes are referenced by unique IDs, can store attributes, and may reference or observe other nodes.
  * When Slicer is saved, the scene is written as an `.mrml` (XML) file, optionally bundled with data in a `.mrb` file.

---

### 2. User Interface Structure

#### **Main UI Components**

| UI Element       | Function                                                                                           |
| ---------------- | -------------------------------------------------------------------------------------------------- |
| **Module Panel** | Left panel showing GUI of the currently active module (selected from the toolbar).                 |
| **Data Probe**   | Bottom of the module panel — displays voxel, segmentation, and slice info under the mouse pointer. |
| **Views**        | Slice, 3D, Chart, and Table views — arranged using layout presets from the Layout Toolbar.         |
| **Toolbars**     | Provide quick access to modules, layouts, favorite modules, and mouse interaction modes.           |
| **Status Bar**   | Shows operation progress, errors, or logs.                                                         |

#### **Application Menu Overview**

* **File:** load/save data and scenes, access sample data, import/export DICOM.
* **Edit:** change app settings (appearance, favorite modules, temp directories, etc.).
* **View:** show/hide toolbars, open Python Console, Extensions Manager, Error Log, etc.
* **Help:** documentation, bug reports, citations, and publication searches.

#### **Data Interaction**

* The **Data module** shows all loaded datasets in a hierarchical tree.
* You can show/hide items with the *eye icon*, drag and drop into specific views, or right-click to access display and transformation options.
* Volume display options include “reset field of view” and “reset orientation on show.”

#### **Cross-Referencing and Mouse Modes**

* **Shift + Mouse Move:** moves the crosshair in all linked slice/3D views.
* **Place Mode / Transform Mode:** toggle between placing markup points and transforming objects.
* Persistent “place” mode allows batch placement of control points.

---

### 3. MRML — Medical Reality Modeling Language

#### **Purpose**

MRML is Slicer’s data model library — an abstraction for all datasets and visualization configurations.

#### **Core Node Types**

| Category                   | Example Node                                                                                                                                            | Description                                                                          |
| -------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| **Data Nodes**             | `vtkMRMLVolumeNode`, `vtkMRMLModelNode`, `vtkMRMLSegmentationNode`, `vtkMRMLMarkupsNode`, `vtkMRMLTransformNode`, `vtkMRMLTextNode`, `vtkMRMLTableNode` | Represent primary datasets (images, surfaces, annotations, transforms, etc.).        |
| **Display Nodes**          | `vtkMRMLDisplayNode` subclasses                                                                                                                         | Control visual properties (color, visibility, rendering style).                      |
| **Storage Nodes**          | `vtkMRMLStorageNode` subclasses                                                                                                                         | Define how data is read/written to disk.                                             |
| **View Nodes**             | `vtkMRMLAbstractViewNode` subclasses                                                                                                                    | Manage view layouts and display configurations.                                      |
| **Subject Hierarchy Node** | `vtkMRMLSubjectHierarchyNode`                                                                                                                           | Organizes data into patient/study/series folders, replacing older hierarchy systems. |
| **Sequence Nodes**         | `vtkMRMLSequenceNode`, `vtkMRMLSequenceBrowserNode`                                                                                                     | Represent temporal or multi-frame datasets.                                          |

---

### 4. MRML Events and Observers

Slicer uses the **VTK event-observer mechanism** to propagate scene and node changes to other nodes or GUI components.

* **Events:** `ModifiedEvent`, `TransformModifiedEvent`, `NodeAddedEvent`, etc.
* **Observation Macros (C++):**

  * `vtkSetMRMLObjectMacro` — registers a MRML object, no observer added.
  * `vtkSetAndObserveMRMLObjectMacro` — registers + observes `ModifiedEvent`.
  * `vtkSetAndObserveMRMLObjectEventsMacro` — registers + observes custom event sets.
* **Common Observer Usage:**

  * Logic and GUI classes implement `ProcessMRMLEvents()` to respond to observed changes.
  * Use `vtkObserverManager` and `vtkEventBroker` to simplify event connections.

---

### 5. Modules Overview

Slicer modules are self-contained functional units that operate on MRML data.
They generally do **not** interact directly — communication happens through the MRML scene.

#### **Types of Modules**

| Type                             | Language                | Characteristics                                                            | Use Case                                           |
| -------------------------------- | ----------------------- | -------------------------------------------------------------------------- | -------------------------------------------------- |
| **Command Line Interface (CLI)** | Any (C++, Python, etc.) | Runs as a separate process with XML-defined I/O. Cannot update UI mid-run. | Computation-heavy algorithms, batch tools.         |
| **Loadable Modules**             | C++                     | Compiled plugins with full API and UI control.                             | Performance-critical or complex interactive logic. |
| **Scripted Modules**             | Python                  | Rapid development using Slicer’s Python API.                               | Prototyping, workflow automation, or extensions.   |

**Module Association with MRML Nodes:**
Modules can declare which MRML node types they edit using:

* `associatedNodeTypes()` (C++ API), or
* `addModuleAssociatedNodeTypes()` (runtime association).

When editing, Slicer activates the module with the highest `nodeEditable()` confidence for the given node.

---

### 6. Parameter Nodes and the Parameter Node Wrapper

**Parameter Nodes** allow modules (especially scripted ones) to persist user parameters and state inside the MRML scene.

* Base class: `vtkMRMLScriptedModuleNode`
* Store key–value pairs (`SetParameter`, `GetParameter`) — string-based by default.

#### **Parameter Node Wrapper (Pythonic Interface)**

To simplify type-safe access and serialization:

```python
import slicer
from slicer.parameterNodeWrapper import *

@parameterNodeWrapper
class CustomParameterNode:
    numIterations: int
    inputs: list[slicer.vtkMRMLModelNode]
    output: slicer.vtkMRMLModelNode
```

Usage:

```python
parameterNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLScriptedModuleNode')
param = CustomParameterNode(parameterNode)
param.numIterations = 500
```

✅ Supports:

* Native Python types (`int`, `float`, `bool`, `str`, `tuple`, etc.)
* MRML nodes and lists thereof
* Default values and callable defaults
* Persistence within `.mrml` / `.mrb` scenes

**Tip:** Use `slicer.<NodeClass>` (not direct imports) when defining wrappers to ensure namespace stability.

---

### 7. Transforms

**Transforms** define spatial mappings between coordinate systems.

* **Transform Nodes:** `vtkMRMLTransformNode` (generic), `vtkMRMLLinearTransformNode`, `vtkMRMLGridTransformNode`, `vtkMRMLBSplineTransformNode`.
* **Transformable Nodes:** any node that can be spatially repositioned (e.g., models, volumes, markups).

#### **Events**

* When a transform node is observed by a transformable node:

  * `TransformModifiedEvent` fires on the transformable node when the transform changes.
  * `ModifiedEvent` fires on the transform node itself.

#### **Usage Notes**

* Transforms can be linear, non-linear, or composite.
* Applying transforms in code:

  ```python
  modelNode.SetAndObserveTransformNodeID(transformNode.GetID())
  ```
* Common examples are available in Slicer’s **Script Repository**.

---

### 8. Developer Resources

Official documentation and examples:

* [Developer Guide: Extensions](https://slicer.readthedocs.io/en/latest/developer_guide/extensions.html#extensions)
* [Developer Tutorials](https://training.slicer.org/#developer-tutorials)
* [Python FAQ](https://slicer.readthedocs.io/en/latest/developer_guide/python_faq.html)
* [Script Repository](https://slicer.readthedocs.io/en/latest/developer_guide/script_repository.html)
* [API Docs (C++ / Python)](https://apidocs.slicer.org/main/classes.html)
* [Scripted Module Examples (GitHub)](https://github.com/Slicer/Slicer/tree/main/Modules/Scripted)

---

## 🔁 Recommended Learning Order

1. **Slicer Basics & Scene Concepts** – understand MRML Scene, nodes, and views.
2. **UI Layout and Navigation** – get comfortable with modules, toolbars, and the Data module.
3. **MRML Node System** – dive into node types, references, attributes, and events.
4. **Modules Overview** – learn the 3 module types and their relationships with MRML.
5. **Parameter Nodes & Wrappers** – practice scripting modules that persist state.
6. **Transforms** – understand how geometry is managed and linked through nodes.
7. **Advanced MRML Events** – respond to scene updates and create reactive scripts.
8. **Development & Extension Writing** – build and distribute scripted modules.
