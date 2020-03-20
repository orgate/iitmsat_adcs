function y = NotEclipse(r_ECI, S_ECI)
if (sum(r_ECI.*S_ECI)>0)
    y=1
else
if (norm(r_ECI-sum(r_ECI.*S_ECI)*S_ECI/sum(S_ECI.*S_ECI))>6400000)
    y=1
else
    y=0
end
end