function safeSetBaudrate(port_num, baudrate, lib_name)
if (setBaudRate(port_num, baudrate))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

end