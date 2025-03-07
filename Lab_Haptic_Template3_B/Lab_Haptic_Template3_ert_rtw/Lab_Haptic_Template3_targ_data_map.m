    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 2;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
            paramMap.sections(nTotSects) = dumSection; %prealloc
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (h1cj1z3bnd)
        ;%
            section.nData     = 4;
            section.data(4)  = dumData; %prealloc

                    ;% h1cj1z3bnd.Delay_InitialCondition
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% h1cj1z3bnd.Delay1_InitialCondition
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% h1cj1z3bnd.Memory_InitialCondition
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% h1cj1z3bnd.Delay2_InitialCondition
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section

            section.nData     = 3;
            section.data(3)  = dumData; %prealloc

                    ;% h1cj1z3bnd.Arduino_P1
                    section.data(1).logicalSrcIdx = 4;
                    section.data(1).dtTransOffset = 0;

                    ;% h1cj1z3bnd.Arduino_P2
                    section.data(2).logicalSrcIdx = 5;
                    section.data(2).dtTransOffset = 1;

                    ;% h1cj1z3bnd.Arduino_P3
                    section.data(3).logicalSrcIdx = 6;
                    section.data(3).dtTransOffset = 2;

            nTotData = nTotData + section.nData;
            paramMap.sections(2) = section;
            clear section


            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (nacmyfsqe10)
        ;%
            section.nData     = 9;
            section.data(9)  = dumData; %prealloc

                    ;% nacmyfsqe10.jh0of0ic15
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% nacmyfsqe10.i1zkcja3hi
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% nacmyfsqe10.nwbgkk2cf1
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% nacmyfsqe10.lo1bxwuqm4
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% nacmyfsqe10.kyc2xayblk
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 4;

                    ;% nacmyfsqe10.of5knslh4d
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 5;

                    ;% nacmyfsqe10.ajw0qehmzm
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 6;

                    ;% nacmyfsqe10.esc5zn0ww0
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 7;

                    ;% nacmyfsqe10.mtfhmaeu44
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 8;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 3;
        sectIdxOffset = 1;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (faahpnajwfr)
        ;%
            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% faahpnajwfr.jw0xwo0d5b
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% faahpnajwfr.psmwnkbxrf
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 3;
            section.data(3)  = dumData; %prealloc

                    ;% faahpnajwfr.aowzeo4gbq.LoggedData
                    section.data(1).logicalSrcIdx = 2;
                    section.data(1).dtTransOffset = 0;

                    ;% faahpnajwfr.gqerfcnbrm.LoggedData
                    section.data(2).logicalSrcIdx = 3;
                    section.data(2).dtTransOffset = 3;

                    ;% faahpnajwfr.nbl13icq1g.LoggedData
                    section.data(3).logicalSrcIdx = 4;
                    section.data(3).dtTransOffset = 4;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 5;
            section.data(5)  = dumData; %prealloc

                    ;% faahpnajwfr.l2q5z0ybvk
                    section.data(1).logicalSrcIdx = 5;
                    section.data(1).dtTransOffset = 0;

                    ;% faahpnajwfr.hqshg21m1k
                    section.data(2).logicalSrcIdx = 6;
                    section.data(2).dtTransOffset = 10;

                    ;% faahpnajwfr.cy2mhgtxvh
                    section.data(3).logicalSrcIdx = 7;
                    section.data(3).dtTransOffset = 20;

                    ;% faahpnajwfr.kd1uwtfzmt
                    section.data(4).logicalSrcIdx = 8;
                    section.data(4).dtTransOffset = 30;

                    ;% faahpnajwfr.o0wjz3lo5a
                    section.data(5).logicalSrcIdx = 9;
                    section.data(5).dtTransOffset = 31;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 4217309502;
    targMap.checksum1 = 632321356;
    targMap.checksum2 = 713394769;
    targMap.checksum3 = 2254048677;

