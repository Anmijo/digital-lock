# Digital-Lock
A complete digital lock implementation on verilog to be flashed on an FPGA

### Project Desctiption
The project was made in my time as a Design Engineer Intern at the STARS semiconductor program. In this project I implement a digital lock design from scratch. The lock in its initialization state allows the user to input an 8 number passkey. After the user is done inputting the key, the user must lock the safe. After that, the safe will remain locked until the correct key is put. If an incorrect key is put, the lock immediately goes into panic mode and alerts the user. The design is filled with visual marks and text on a seven segment display to create a better user interface.

### Design Process
The design process included drawing mealy state machines to create the logic for the Finite State Machines used to implement the design. The design also involves a button key synchronizer to eliminate metastability with button presses. Seven segement displays are used to visually display everything to the user.

### Modules Used
FSM: Finite state machine module to handle logic
SSDEC: Handle wiring of seven segment displays
Synckey: Key synchronizer to eliminate button-press metastability
Display: Handle logic between FSM and seven segment displays.
