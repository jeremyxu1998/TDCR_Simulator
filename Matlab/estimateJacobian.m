function J = estimateJacobian(q, param, epsilon)

J = zeros(6, length(q));

T_q = Forward(q, param);
for i = 1:length(q)
    qi = q;
    qi(i) = qi(i) + epsilon;
    T_qi = Forward(qi, param);
    
    Ji_hom = invT(T_q)*(T_qi - T_q)/epsilon;
    vB = Ji_hom(1:3,4);
    wB = [Ji_hom(3,2), Ji_hom(1,3), Ji_hom(2,1)]';
    
    J(:,i) = [wB; vB];
end

