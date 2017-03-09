function flg = Within_Range(qnum, range_min, range_max)

if qnum >= range_min && qnum <= range_max
  flg = true;
else
  flg = false;
end

end