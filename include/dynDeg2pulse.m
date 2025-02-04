function pulse = dynDeg2pulse(deg)
    deg = wrapTo360(deg);
    pulse = deg*4096/360;
    pulse = typecast(int32(pulse), 'uint32');
end