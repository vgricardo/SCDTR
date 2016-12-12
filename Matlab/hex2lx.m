function [ lx ] = hex2lx( s )
%hex2lx Converte de hexadecimal para lux
    RBASE = 65078.575;
    EXPO = 1.4241;
    s = strrep(s, '\n', '');
    v = hex2dec(s)*5/1024;
    lx = (RBASE/(50000/v-10000))^EXPO;
end

