MODULE SERVER

!////////////////
!GLOBAL VARIABLES
!////////////////

!//Robot configuration
PERS tooldata currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
PERS wobjdata currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];   
PERS speeddata currentSpeed;
PERS num currentMoveTime;
PERS zonedata currentZone;

!// Clock Synchronization
PERS bool startLog:=TRUE;
PERS bool startRob:=TRUE;

!// Mutex between logger and changing the tool and work objects
PERS bool frameMutex:=FALSE;

!//PC communication
VAR socketdev clientSocket;
VAR socketdev serverSocket;
VAR num instructionCode;
VAR num params{30};
VAR num nParams;

PERS string ipController:= "192.168.125.1"; !robot default IP
PERS num serverPort:= 5000;

!//Motion of the robot
VAR robtarget cartesianTarget;
VAR jointtarget jointsTarget;
VAR bool moveCompleted; !Set to true after finishing a Move instruction.

!//Buffered move variables
CONST num MAX_BUFFER := 2048;
VAR num BUFFER_POS := 0;
VAR num BUFFER_JOINT_POS :=0;
VAR num BUFFER_JOINT_TIME_POS :=0;
VAR robtarget bufferTargets{MAX_BUFFER};
VAR speeddata bufferSpeeds{MAX_BUFFER};
VAR jointtarget bufferJointPos{MAX_BUFFER};
VAR speeddata bufferJointSpeeds{MAX_BUFFER};
VAR jointtarget bufferJointTimePos{MAX_BUFFER};
VAR num bufferJointTimes{MAX_BUFFER};

!//External axis position variables
VAR extjoint externalAxis;

!//Circular move buffer
VAR robtarget circPoint;

!//Correct Instruction Execution and possible return values
VAR num ok;
CONST num SERVER_BAD_MSG :=  0;
CONST num SERVER_OK := 1;



    
!////////////////
!LOCAL METHODS
!////////////////

!//Method to parse the message received from a PC
!// If correct message, loads values on:
!// - instructionCode.
!// - nParams: Number of received parameters.
!// - params{nParams}: Vector of received params.
PROC ParseMsg(string msg)
    !//Local variables
    VAR bool auxOk;
    VAR num ind:=1;
    VAR num newInd;
    VAR num length;
    VAR num indParam:=1;
    VAR string subString;
    VAR bool end := FALSE;
    
    !//Find the end character
    length := StrMatch(msg,1,"#");
    IF length > StrLen(msg) THEN
        !//Corrupt message
        nParams := -1;
    ELSE
        !//Read Instruction code
        newInd := StrMatch(msg,ind," ") + 1;
        subString := StrPart(msg,ind,newInd - ind - 1);
        auxOk:= StrToVal(subString, instructionCode);
        IF auxOk = FALSE THEN
            !//Impossible to read instruction code
            nParams := -1;
        ELSE
            ind := newInd;
            !//Read all instruction parameters (maximum of 8)
            WHILE end = FALSE DO
                newInd := StrMatch(msg,ind," ") + 1;
                IF newInd > length THEN
                    end := TRUE;
                ELSE
                    subString := StrPart(msg,ind,newInd - ind - 1);
                    auxOk := StrToVal(subString, params{indParam});
                    indParam := indParam + 1;
                    ind := newInd;
                ENDIF      
            ENDWHILE
            nParams:= indParam - 1;
        ENDIF
    ENDIF
ENDPROC


!//Handshake between server and client:
!// - Creates socket.
!// - Waits for incoming TCP connection.
PROC ServerCreateAndConnect(string ip, num port)
    VAR string clientIP;
    
    SocketCreate serverSocket;
    SocketBind serverSocket, ip, port;
    SocketListen serverSocket;
    TPWrite "SERVER: Server waiting for incoming connections ...";
    WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
        SocketAccept serverSocket,clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
        IF SocketGetStatus(clientSocket) <> SOCKET_CONNECTED THEN
            TPWrite "SERVER: Problem serving an incoming connection.";
            TPWrite "SERVER: Try reconnecting.";
        ENDIF
        !//Wait 0.5 seconds for the next reconnection
        WaitTime 0.5;
    ENDWHILE
    TPWrite "SERVER: Connected to IP " + clientIP;
ENDPROC


!//Parameter initialization
!// Loads default values for
!// - Tool.
!// - WorkObject.
!// - Zone.
!// - Speed.
PROC Initialize()
    currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
    currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    currentSpeed := [150, 50, 0, 0];
    currentMoveTime := 3;
    currentZone := [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03]; !z0
    
    !Find the current external axis values so they don't move when we start
    jointsTarget := CJointT();
    externalAxis := jointsTarget.extax;
ENDPROC


!////////////////////////
!//SERVER: Main procedure
!////////////////////////
PROC main()
    !//Local variables
    VAR string receivedString;   !//Received string
    VAR string sendString;       !//Reply string
    VAR string addString;        !//String to add to the reply.
    VAR bool connected;          !//Client connected
    VAR bool reconnected;        !//Drop and reconnection happened during serving a command
    VAR robtarget cartesianPose;
    VAR robtarget toPoint;
    VAR jointtarget jointsPose;
    VAR num SensorX:=1;
    VAR num SensorY:=2;
    VAR num SensorZ:=3;
    VAR num SensorWX:=4;
    VAR num SensorWY:=5;
    VAR num SensorWZ:=6;
    VAR num ForceX:=7;
    VAR num ForceY:=8;
    VAR num ForceZ:=9;
    VAR num ForceWX:=10;
    VAR num ForceWY:=11;
    VAR num ForceWZ:=12;

    VAR num startFT{6};
    VAR num keepGoing;
    VAR num ftThreshX; 
    VAR num ftThreshY; 
    VAR num ftThreshZ; 
    
    VAR num i;
    
    VAR loaddata TestLoad:=[3.0,[0,0,0.015],[1,0,0,0],0,0,0];

    !//Motion configuration
    ConfL \Off;
    SingArea \Wrist;
    moveCompleted:= TRUE;
    
    !//Initialization of WorkObject, Tool, Speed and Zone
    Initialize;

    !//Socket connection
    connected:=FALSE;
    ServerCreateAndConnect ipController,serverPort; 
    connected:=TRUE;
    
    !//Torque Sensor Definitions AC
    TestSignDefine SensorX, 201, ROB_1, 1, 0;
    TestSignDefine SensorY, 202, ROB_1, 1, 0;
    TestSignDefine SensorZ, 203, ROB_1, 1, 0;
    TestSignDefine SensorWX, 204, ROB_1, 1, 0;
    TestSignDefine SensorWY, 205, ROB_1, 1, 0;
    TestSignDefine SensorWZ, 206, ROB_1, 1, 0;
    TestSignDefine ForceX, 207, ROB_1, 1, 0;
    TestSignDefine ForceY, 208, ROB_1, 1, 0;
    TestSignDefine ForceZ, 209, ROB_1, 1, 0;
    TestSignDefine ForceWX, 210, ROB_1, 1, 0;
    TestSignDefine ForceWY, 211, ROB_1, 1, 0;
    TestSignDefine ForceWZ, 212, ROB_1, 1, 0;
    
    !//Server Loop
    WHILE TRUE DO
        !//Initialization of program flow variables
        ok:=SERVER_OK;              !//Correctness of executed instruction.
        reconnected:=FALSE;         !//Has communication dropped after receiving a command?
        addString := "";            

        !//Wait for a command
        SocketReceive clientSocket \Str:=receivedString \Time:=WAIT_MAX;
        ParseMsg receivedString;
    
        !//Execution of the command
        TEST instructionCode
            CASE 0: !Ping
                IF nParams = 0 THEN
                    ok := SERVER_OK;
                ELSE
                    ok := SERVER_BAD_MSG;
                ENDIF

            CASE 1: !Cartesian Move
                IF nParams = 7 THEN
                    cartesianTarget :=[[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       [0,0,0,0],
                                       externalAxis];
                    ok := SERVER_OK;
                    moveCompleted := FALSE;
                    MoveL cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj ;
                    moveCompleted := TRUE;
                ELSE
                    ok := SERVER_BAD_MSG;
                ENDIF   
                
            CASE 2: !Joint Move
                IF nParams = 6 THEN
                    jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}], externalAxis];
                    ok := SERVER_OK;
                    moveCompleted := FALSE;
                    MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
                    moveCompleted := TRUE;
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF

            CASE 3: !Get Cartesian Coordinates (with current tool and workobject)
                IF nParams = 0 THEN
                    cartesianPose := CRobT(\Tool:=currentTool \WObj:=currentWObj);      
                    addString := NumToStr(cartesianPose.trans.x,2) + " ";
                    addString := addString + NumToStr(cartesianPose.trans.y,2) + " ";
                    addString := addString + NumToStr(cartesianPose.trans.z,2) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q1,3) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q2,3) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q3,3) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q4,3); !End of string   
                    ok := SERVER_OK;
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF

            CASE 4: !Get Joint Coordinates
                IF nParams = 0 THEN
                    jointsPose := CJointT();
                    addString := NumToStr(jointsPose.robax.rax_1,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_2,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_3,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_4,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_5,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_6,2); !End of string
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 5: !Get external axis positions
                IF nParams = 0 THEN
                    jointsPose := CJointT();
                    addString := StrPart(NumToStr(jointsTarget.extax.eax_a, 2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_b,2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_c,2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_d,2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_e,2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_f,2),1,8); !End of string
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF   
        
            CASE 6: !Set Tool
                IF nParams = 7 THEN
                    WHILE (frameMutex) DO
                        WaitTime .01; !// If the frame is being used by logger, wait here
                    ENDWHILE
                    !//frameMutex:= TRUE;
                    currentTool.tframe.trans.x:=params{1};
                    currentTool.tframe.trans.y:=params{2};
                    currentTool.tframe.trans.z:=params{3};
                    currentTool.tframe.rot.q1:=params{4};
                    currentTool.tframe.rot.q2:=params{5};
                    currentTool.tframe.rot.q3:=params{6};
                    currentTool.tframe.rot.q4:=params{7};
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 7: !Set Work Object
                IF nParams = 7 THEN
                    currentWobj.oframe.trans.x:=params{1};
                    currentWobj.oframe.trans.y:=params{2};
                    currentWobj.oframe.trans.z:=params{3};
                    currentWobj.oframe.rot.q1:=params{4};
                    currentWobj.oframe.rot.q2:=params{5};
                    currentWobj.oframe.rot.q3:=params{6};
                    currentWobj.oframe.rot.q4:=params{7};
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 8: !Set Speed of the Robot
                IF nParams = 4 THEN
                    currentSpeed.v_tcp:=params{1};
                    currentSpeed.v_ori:=params{2};
                    currentSpeed.v_leax:=params{3};
                    currentSpeed.v_reax:=params{4};
                    ok := SERVER_OK;
                ELSEIF nParams = 2 THEN
                    currentSpeed.v_tcp:=params{1};
                    currentSpeed.v_ori:=params{2};
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 9: !Set zone data
                IF nParams = 4 THEN
                    IF params{1}=1 THEN
                        currentZone.finep := TRUE;
                        currentZone.pzone_tcp := 0.0;
                        currentZone.pzone_ori := 0.0;
                        currentZone.zone_ori := 0.0;
                    ELSE
                        currentZone.finep := FALSE;
                        currentZone.pzone_tcp := params{2};
                        currentZone.pzone_ori := params{3};
                        currentZone.zone_ori := params{4};
                    ENDIF
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 10: !Get Force-Torque Readings  -AC 2/16/2015
                IF nParams = 0 THEN
                    addString := NumToStr(TestSignRead(SensorX),2) + " ";
                    addString := addString + NumToStr(TestSignRead(SensorY),2) + " ";
                    addString := addString + NumToStr(TestSignRead(SensorZ),2) + " ";
                    addString := addString + NumToStr(TestSignRead(SensorWX),2) + " ";
                    addString := addString + NumToStr(TestSignRead(SensorWY),2) + " ";
                    addString := addString + NumToStr(TestSignRead(SensorWZ),2) + " "; !End of string
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 11: !Get Force-Torque Readings Part2  -AC 2/16/2015
                IF nParams = 0 THEN
                    addString := NumToStr(TestSignRead(ForceX),2) + " ";
                    addString := addString + NumToStr(TestSignRead(ForceY),2) + " ";
                    addString := addString + NumToStr(TestSignRead(ForceZ),2) + " ";
                    addString := addString + NumToStr(TestSignRead(ForceWX),2) + " ";
                    addString := addString + NumToStr(TestSignRead(ForceWY),2) + " ";
                    addString := addString + NumToStr(TestSignRead(ForceWZ),2) + " "; !End of string
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 30: !Add Cartesian Coordinates to buffer
                IF nParams = 7 THEN
                    cartesianTarget :=[[params{1},params{2},params{3}],
                                        [params{4},params{5},params{6},params{7}],
                                        [0,0,0,0],
                                        externalAxis];
                    IF BUFFER_POS < MAX_BUFFER THEN
                        BUFFER_POS := BUFFER_POS + 1;
                        bufferTargets{BUFFER_POS} := cartesianTarget;
                        bufferSpeeds{BUFFER_POS} := currentSpeed;
                    ENDIF
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 31: !Clear Cartesian Buffer
                IF nParams = 0 THEN
                    BUFFER_POS := 0;    
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 32: !Get Buffer Size)
                IF nParams = 0 THEN
                    addString := NumToStr(BUFFER_POS,2);
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 33: !Execute moves in cartesianBuffer as linear moves
                IF nParams = 0 THEN
                    FOR i FROM 1 TO (BUFFER_POS) DO 
                        MoveL bufferTargets{i}, bufferSpeeds{i}, currentZone, currentTool \WObj:=currentWobj ;
                    ENDFOR          
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 34: !External Axis move
                IF nParams = 6 THEN
                    externalAxis :=[params{1},params{2},params{3},params{4},params{5},params{6}];
                    jointsTarget := CJointT();
                    jointsTarget.extax := externalAxis;
                    ok := SERVER_OK;
                    moveCompleted := FALSE;
                    MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
                    moveCompleted := TRUE;
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF

            CASE 35: !Specify circPoint for circular move, and then wait on toPoint
                IF nParams = 7 THEN
                    circPoint :=[[params{1},params{2},params{3}],
                                [params{4},params{5},params{6},params{7}],
                                [0,0,0,0],
                                externalAxis];
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 36: !specify toPoint, and use circPoint specified previously
                IF nParams = 7 THEN
                    cartesianTarget :=[[params{1},params{2},params{3}],
                                        [params{4},params{5},params{6},params{7}],
                                        [0,0,0,0],
                                        externalAxis];
                    MoveC circPoint, cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj ;
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

        CASE 37: !Add Joint Positions to buffer
            IF nParams = 6 THEN
            jointsTarget := [[params{1},params{2},params{3},params{4},params{5},params{6}],externalAxis];
            IF BUFFER_JOINT_POS < MAX_BUFFER THEN
                BUFFER_JOINT_POS := BUFFER_JOINT_POS + 1;
                bufferJointPos{BUFFER_JOINT_POS} :=jointsTarget;
                bufferJointSpeeds{BUFFER_JOINT_POS} := currentSpeed;
            ENDIF
            ok := SERVER_OK;
        ELSE
            ok := SERVER_BAD_MSG;
        ENDIF

        CASE 38: !Clear Joint Position Buffer
            IF nParams = 0 THEN
            BUFFER_JOINT_POS := 0;
            ok :=SERVER_OK;
        ELSE
            ok := SERVER_BAD_MSG;
        ENDIF

        CASE 39: !Get Joint Position Buffer Size
            IF nParams = 0 THEN
            addString:= NumToStr(BUFFER_JOINT_POS,2);
            ok := SERVER_OK;
        ELSE
            ok := SERVER_BAD_MSG;
        ENDIF    

        CASE 40: !Execute moves in bufferJointPos
            IF nParams = 0 THEN
            IF BUFFER_JOINT_POS > 0 THEN
            FOR i FROM 1 TO (BUFFER_JOINT_POS) DO
                MoveAbsJ bufferJointPos{i}, bufferJointSpeeds{i}, currentZone, currentTool, \Wobj:=currentWobj;
            ENDFOR
            ENDIF
            ok :=SERVER_OK;
        ELSE
                ok :=SERVER_BAD_MSG;
        ENDIF


        CASE 41: !Set time the robot takes to make its next move
                IF nParams = 1 THEN
                    currentMoveTime:=params{1};
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

        CASE 42: !Add Joint Positions to time-controlled buffer
            IF nParams = 6 THEN
            jointsTarget := [[params{1},params{2},params{3},params{4},params{5},params{6}],externalAxis];
            IF BUFFER_JOINT_TIME_POS < MAX_BUFFER THEN
                BUFFER_JOINT_TIME_POS := BUFFER_JOINT_TIME_POS + 1;
            bufferJointTimePos{BUFFER_JOINT_TIME_POS} :=jointsTarget;
            bufferJointTimes{BUFFER_JOINT_TIME_POS} := currentMoveTime;
            ENDIF
            ok := SERVER_OK;
        ELSE
            ok := SERVER_BAD_MSG;
        ENDIF

        CASE 43: !Clear Joint Position Time Buffer
            IF nParams = 0 THEN
            BUFFER_JOINT_TIME_POS := 0;
            ok :=SERVER_OK;
        ELSE
            ok := SERVER_BAD_MSG;
        ENDIF

        CASE 44: !Get Joint Position Time Buffer Size
            IF nParams = 0 THEN
            addString:= NumToStr(BUFFER_JOINT_TIME_POS,2);
            ok := SERVER_OK;
        ELSE
            ok := SERVER_BAD_MSG;
        ENDIF

        CASE 45: !Execute moves in bufferJointTimePos using bufferJointTimes
            IF nParams = 0 THEN
                IF BUFFER_JOINT_TIME_POS > 0 THEN
                    MoveAbsJ bufferJointTimePos{1}, currentSpeed, currentZone, currentTool, \Wobj:=currentWobj;
                    IF BUFFER_JOINT_TIME_POS > 1 THEN
                    FOR i FROM 2 TO (BUFFER_JOINT_TIME_POS) DO
                        MoveAbsJ bufferJointTimePos{i}, currentSpeed, \T:=bufferJointTimes{i}, currentZone, currentTool, \Wobj:=currentWobj;
                    ENDFOR
                    ENDIF
                ENDIF
                ok :=SERVER_OK;
            ELSE
                ok :=SERVER_BAD_MSG;
            ENDIF


        CASE 46: !Activate soft servo
            IF nParams = 3 THEN
                SoftAct params{1}, params{2};
                ok :=SERVER_OK;
            ELSE 
                ok :=SERVER_BAD_MSG;
            ENDIF

        CASE 47: !Deactivate soft servo
            IF nParams = 0 THEN
                SoftDeact;
                ok :=SERVER_OK;
            ELSE 
                ok :=SERVER_BAD_MSG;
            ENDIF

        CASE 48: !Calibrate force sensor at current orientation
            IF nParams = 0 THEN
                FCCalib TestLoad;
                ok :=SERVER_OK;
            ELSE 
                ok :=SERVER_BAD_MSG;
            ENDIF

        CASE 49: !Start pressing on a surface using point specified previously
            IF nParams = 4 THEN
                !toPoint := [[params{1},params{2},params{3}],
                !                        [params{4},params{5},params{6},params{7}],
                !                        [0,0,0,0],
                !                        externalAxis];
                ! destination point, speed, Fx y and z, force threshold (integer 0-100), 
                ! starts at current point, applies force Fx / y / z * force_threshold until equivalent force is
                ! met. Then moves towards destination point, ramping up to full force.
                FCPress1LStart circPoint, currentSpeed, \Fx:=params{1}, \Fy:=params{2}, \Fz:=params{3}, params{4}, currentZone, currentTool;
                ok :=SERVER_OK;
            ELSE 
                ok :=SERVER_BAD_MSG;
            ENDIF


        CASE 50: !Stop pressing on surface
            IF nParams = 7 THEN
                toPoint := [[params{1},params{2},params{3}],
                                        [params{4},params{5},params{6},params{7}],
                                        [0,0,0,0],
                                        externalAxis];
                FCPressEnd toPoint, currentSpeed, currentTool;
                ok :=SERVER_OK;
            ELSE 
                ok :=SERVER_BAD_MSG;
            ENDIF

        CASE 51: !do press calibration
            IF nParams = 0 THEN
                !// Initialize for force control
                !// Identify load using the sensor and the current orientation
                TestLoad:=FCLoadID();
                ok :=SERVER_OK;
            ELSE 
                ok :=SERVER_BAD_MSG;
            ENDIF

        CASE 52: ! Save FT for buffer
            IF nParams = 0 THEN
                startFT{1} := TestSignRead(SensorX);
                startFT{2} := TestSignRead(SensorY);
                startFT{3} := TestSignRead(SensorZ);
                startFT{4} := TestSignRead(SensorWX);
                startFT{5} := TestSignRead(SensorWY);
                startFT{6} := TestSignRead(SensorWZ);
                ok :=SERVER_OK;
            ELSE
                ok :=SERVER_BAD_MSG;
            ENDIF

        CASE 53: !Execute moves in bufferJointPos, querying forcetorque between steps and
                 ! stopping if it exceeds a threshold
            IF nParams = 0 THEN
                IF BUFFER_JOINT_POS > 0 THEN
                    keepGoing := 1;
                    ftThreshX := 5;
                    ftThreshY := 5;
                    ftThreshZ := 5;
                    i := 1;
                    
                    WHILE i <= BUFFER_JOINT_POS AND keepGoing = 1 DO
                        IF TestSignRead(SensorX) > startFT{1} + ftThreshX OR
                           TestSignRead(SensorX) < startFT{1} - ftThreshX OR
                           TestSignRead(SensorY) > startFT{2} + ftThreshY OR
                           TestSignRead(SensorY) < startFT{2} - ftThreshY OR
                           TestSignRead(SensorZ) > startFT{3} + ftThreshZ OR
                           TestSignRead(SensorZ) < startFT{3} - ftThreshZ THEN
                           keepGoing := 0;
                        ELSE
                            MoveAbsJ bufferJointPos{i}, bufferJointSpeeds{i}, currentZone, currentTool, \Wobj:=currentWobj;
                            i := i + 1;
                        ENDIF
                    ENDWHILE
                ENDIF
                ok :=SERVER_OK;
            ELSE
                    ok :=SERVER_BAD_MSG;
            ENDIF

        CASE 98: !returns current robot info: serial number, robotware version, and robot type
            IF nParams = 0 THEN
                addString := GetSysInfo(\SerialNo) + "*";
                addString := addString + GetSysInfo(\SWVersion) + "*";
                addString := addString + GetSysInfo(\RobotType);
                ok := SERVER_OK;
            ELSE
                ok :=SERVER_BAD_MSG;
            ENDIF

        CASE 99: !Close Connection
            IF nParams = 0 THEN
                TPWrite "SERVER: Client has closed connection.";
                connected := FALSE;
                !//Closing the server
                SocketClose clientSocket;
                SocketClose serverSocket;

                !Reinitiate the server
                ServerCreateAndConnect ipController,serverPort;
                connected := TRUE;
                reconnected := TRUE;
                ok := SERVER_OK;
            ELSE
                ok := SERVER_BAD_MSG;
            ENDIF

        DEFAULT:
            TPWrite "SERVER: Illegal instruction code";
            ok := SERVER_BAD_MSG;

        ENDTEST
        
        !Compose the acknowledge string to send back to the client
        IF connected = TRUE THEN
            IF reconnected = FALSE THEN
                IF SocketGetStatus(clientSocket) = SOCKET_CONNECTED THEN
                    sendString := NumToStr(instructionCode,0);
                    sendString := sendString + " " + NumToStr(ok,0);
                    sendString := sendString + " " + addString;
                    SocketSend clientSocket \Str:=sendString;
                ENDIF
            ENDIF
        ENDIF
    ENDWHILE

ERROR (LONG_JMP_ALL_ERR)
    TPWrite "SERVER: ------";
    TPWrite "SERVER: Error Handler:" + NumtoStr(ERRNO,0);
    TEST ERRNO
        CASE ERR_SOCK_CLOSED:
            TPWrite "SERVER: Lost connection to the client.";
            TPWrite "SERVER: Closing socket and restarting.";
            TPWrite "SERVER: ------";
            connected:=FALSE;
            !//Closing the server
            SocketClose clientSocket;
            SocketClose serverSocket;
            !//Reinitiate the server
            ServerCreateAndConnect ipController,serverPort;
            reconnected:= TRUE;
            connected:= TRUE;
            RETRY; 
        DEFAULT:
            TPWrite "SERVER: Unknown error.";
            TPWrite "SERVER: Closing socket and restarting.";
            TPWrite "SERVER: ------";
            connected:=FALSE;
            !//Closing the server
            SocketClose clientSocket;
            SocketClose serverSocket;
            !//Reinitiate the server
            ServerCreateAndConnect ipController,serverPort;
            reconnected:= TRUE;
            connected:= TRUE;
            RETRY;
    ENDTEST
ENDPROC

ENDMODULE
