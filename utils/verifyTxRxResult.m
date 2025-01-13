function [dxl_comm_result, dxl_error] = verifyTxRxResult(port_num, PROTOCOL_VERSION)
    
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

end