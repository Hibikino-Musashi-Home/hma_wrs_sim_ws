# sm_cu_node
This node provides the state machine's function to execute the Clean Up task.

## State machine
<img src="resources/state_machine.png"/><br>
This state machine consists of the following states.

- [Init](#init)
- [Wait4Start](#wait4start) 
- [Start](#start)
- [OpenDrawer](#opendrawer)
- [Mapping](#mapping)
- [GoToRecPlace](#gotorecplace)
- [GetObject](#getobject)
- [GetObjectFromUnder](#getobjectfromunder)
- [PlaceObject](#placeobject)
- [End](#end)
- [Except](#except)
- [Exit](#exit)

### Init
Initialize state.

**Transitions**
- `next`: [Wait4Start](#wait4start)
    - Transition when initialization is complete.
- `except`: [Except](#except)
    - Transition when an exception occurs.

### Wait4Start
Wait for start state.

**Transitions**
- `next`: [Start](#start)
    - Transition when ready to start.
- `except`: [Except](#except)
    - Transition when an exception occurs.

### Start
Start state.

**Transitions**
- `next`: [OpenDrawer](#opendrawer)
    - Transition when start begins.
- `except`: [Except](#except)
    - Transition when an exception occurs.

### OpenDrawer
Open drawer state.

**Transitions**
- `next`: [Mapping](#mapping)
    - Transition when all Drawers are opened.
- `except`: [Except](#except)
    - Transition when an exception occurs.

### Mapping
Mapping object state.

**Transitions**
- `next`: [GoToRecPlace](#gotorecplace)
    - Transition when mapping is complete.
- `except`: [Except](#except)
    - Transition when an exception occurs.

### GoToRecPlace
Go to recognition place state.

**Transitions**
- `next`: [GetObject](#getobject)
    - Transition when the robot moves to the recognition place (floor or tables).
- `under`: [GetObjectFromUnder](#getobjectfromunder)
    - Transition when the robot moves to the recognition place (under the tables).
- `mapping`: [Mapping](#mapping)
    - Transition when there is no more data stored in the mapping data.
- `loop`: [GoToRecPlace](#gotorecplace)
    - Transition when there is no more data stored in the mapping data on the floor.
- `except`: [Except](#except)
    - Transition when an exception occurs.

### GetObject
Get object state.

**Transitions**
- `next`: [PlaceObject](#placeobject)
    - Transition when the robot succeeds in grasping the object.
- `move`: [GoToRecPlace](#gotorecplace)
    - Transition when the robot fails to grasp the object.
- `loop`: [GetObject](#getobject)
    - Transition when the robot fails to grasp the object (Not implemented).
- `except`: [Except](#except)
    - Transition when an exception occurs.

### GetObjectFromUnder
Get object from under the table state.

**Transitions**
- `next`: [PlaceObject](#placeobject)
    - Transition when the robot succeeds in grasping the object.
- `move`: [GoToRecPlace](#gotorecplace)
    - Transition when the robot fails to grasp the object.
- `loop`: [GetObjectFromUnder](#getobjectfromunder)
    - Transition when the robot fails to grasp the object (Not implemented).
- `except`: [Except](#except)
    - Transition when an exception occurs.

### PlaceObject
Place object to deposit place state.

**Transitions**
- `next`: [GoToRecPlace](#gotorecplace)
    - Transition when object place is complete.
- `except`: [Except](#except)
    - Transition when an exception occurs.

### End
End state.

**Transitions**
- `end`: [Exit](#exit)
    - Transition when ready to exit.

### Except
Except state.

**Transitions**
- `except`: [Exit](#exit)
    - Transition when ready to exit.

### Exit
Exit state.