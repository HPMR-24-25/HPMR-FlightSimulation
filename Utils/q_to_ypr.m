function ypr = q_to_ypr(q)
Q = dcm_from_q(q);
ypr = dcm_to_ypr(Q);
end 