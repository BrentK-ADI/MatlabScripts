%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright © 2023 by Analog Devices, Inc.  All rights reserved.
%
% This software is proprietary to Analog Devices, Inc. and its licensors.
%
% This software is provided on an “as is” basis without any representations,
% warranties, guarantees or liability of any kind.
%
% Use of the software is subject to the terms and conditions of the
% Clear BSD License ( https://spdx.org/licenses/BSD-3-Clause-Clear.html ).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef linduinoSPI
    %linduinoSPI - Simple class for communicating with a DC590B or DC2026 for
    %              SPI communications. Will automatically detect the device
    %              upon creation.  Note: The linduinoSPI instance must be
    %              cleared when finished to correctly close out the serial port

    properties (Access = private)
        portConn    %Serial port connection
    end

    methods
        function self = linduinoSPI()
        % linduinoSPI - Class constructor. Attempts to find a board through the
        %               available serial ports

            self.portConn = [];

            availPorts = serialportlist("available")
            for port = availPorts %Loop through all available ports
                s = serialport(port, 115200, "Timeout",1);
                pause(2); %Give it time to connect
                flush(s); %Clear out any linger Rx data just in case

                write(s,'i','char'); %Send the 'i' command to inspect the board
                id_str = readline(s);%Read the response

                %The response will be a length string with DC590 at 21:25
                if(~isempty(id_str) && strlength(id_str) > 24 && ...
                       strcmp(extractBetween(id_str,21,25), 'DC590'))
                    self.portConn = s;
                    fprintf('Port %s appears to be a DC590 or Linduino\n', port);
                    return;
                else
                    clear s
                end
            end
            fprintf('Did not find a DC590 or Linduino');
        end

        function connected = isConnected(self)
        % isConnected - Returns if the instance is connected to a port

            connected = ~isempty(self.portConn);
            return;
        end

        function delete(self)
        % delete - Class destructor. Clean up the comm port

            clear self.portConn
        end

        function retDat = writeRead(self, data)
        %writeRead - Performs a SPI write and read operation. The data passed in
        %            should be an array of bytes to transmit. The return array
        %            will be of equal lenght with the received data
        % Inputs
        %   data - Array of bytes to transmit
        %
        % Outputs
        %   array of bytes received. Same length as data input
            retDat = [];
            ctrlStr = "x";  %CS Low
            for byte = data
                %Append all bytes with T for transaction, then ASCII hex
                ctrlStr = strcat(ctrlStr, sprintf("T%02X", uint8(byte)));
            end
            ctrlStr = strcat(ctrlStr, "XZ"); %CS High and a new line
            writeline(self.portConn, ctrlStr);
            resultStr = char(readline(self.portConn));

            if(length(resultStr) < (2*length(data)))
                fprintf("Length mismatch on read: %d %d\n", ...
                    length(resultStr), 2*length(data));
            else
                for i = 1:length(data)
                    retDat(i) = hex2dec(resultStr((i*2)-1:((i*2))));
                end
            end

            return;
        end
    end
end

