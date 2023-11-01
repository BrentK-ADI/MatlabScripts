%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2023 Analog Devices, Inc.
%
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%  - Redistributions of source code must retain the above copyright
%    notice, this list of conditions and the following disclaimer.
%  - Redistributions in binary form must reproduce the above copyright
%    notice, this list of conditions and the following disclaimer in
%    the documentation and/or other materials provided with the
%    distribution.
%  - Neither the name of Analog Devices, Inc. nor the names of its
%    contributors may be used to endorse or promote products derived
%    from this software without specific prior written permission.
%  - The use of this software may or may not infringe the patent rights
%    of one or more patent holders.  This license does not release you
%    from the requirement that you obtain separate licenses from these
%    patent holders to use this software.
%  - Use of the software either in source or binary form, must be run
%    on or directly connected to an Analog Devices Inc. component.
%
% THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
% IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
% MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
% IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
% LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

