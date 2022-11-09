function theta = getAngle(q1, q2, lambda)

if isempty(lambda)
    Z = quatmultiply(quatconjugate(q1),q2);   
else
    l = repmat(lambda,size(q1,1),1);
    Z = quatmultiply(quatmultiply(quatconjugate(q1),q2),l);
end

theta = 2*acosd(Z(:,1));