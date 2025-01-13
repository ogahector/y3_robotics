function in_range = inRange(val, setpoint, err_perc)

in_range = ~((val > setpoint*(1+err_perc/100)) | (val < setpoint*(1-err_perc/100)));

end
