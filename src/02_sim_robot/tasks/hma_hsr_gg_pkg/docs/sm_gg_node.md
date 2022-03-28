# sm_gg_node
This node provides the state machine's function to execute the Go and Get it task.

## Overview
<img src="resources/state_machine.png"/><br>
This state machine consists of the following states.

- [Init](#init)
- [Wait4Start](#wait4start) 
- [Start](#start)
- [GoToRecPlace](#gotorecplace)
- [EnterToRoom2](#entertoroom2)
- [GoToShelf](#gotoshelf)
- [GetObject](#getobject)
- [DeliverObject](#deliverobject)
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

### GoToRecPlace
Go to recognition place state.

**Transitions**
- `next`: [EnterToRoom2](#entertoroom2)
    - Transition when the robot moves to the recognition place of the obstacle on the floor.
- `except`: [Except](#except)
    - Transition when an exception occurs.

### EnterToRoom2
Enter to Room 2 state.

**Transitions**
- `next`: [GoToShelf](#gotoshelf)
    - Transition when the robot succeeds entered Room 2 (Task 2a is completed).
- `loop`: [EnterToRoom2](#entertoroom2)
    - Transition when the robot fails to enter Room 2.
- `except`: [Except](#except)
    - Transition when an exception occurs.

### GoToShelf
Go to shelf state.

**Transitions**
- `next`: [GetObject](#getobject)
    - Transition when the robot moves to the shelf.
- `except`: [Except](#except)
    - Transition when an exception occurs.

### GetObject
Get object from the shelf state.

**Transitions**
- `next`: [DeliverObject](#deliverobject)
    - Transitions when the specified object is grasped.
- `move`: [GoToShelf](#gotoshelf)
    - Transitions when objects are not recognized.
- `loop`: [GetObject](#getobject)
    - Transitions when the specified object is NOT grasped.
- `except`: [Except](#except)
    - Transition when an exception occurs.

### DeliverObject
Deliver the object to designated person state.

**Transitions**
- `next`: [GoToShelf](#gotoshelf)
    - Transition when delivery is complete.
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