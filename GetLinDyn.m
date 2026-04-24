function [A_lin,B_lin] = GetLinDyn(M_k,m_k,l_k)
    g = 9.81;
    
    A_lin = [0,          1,              0, 0;
             0,          0,        -(m_k*g)/M_k, 0;
             0,          0,              0, 1;
             0,          0,  (M_k+m_k)*g/(M_k*l_k), 0];

    % M_k = 30;
    % m_k = 20;
    % l_k = 3.4;
    
    B_lin = [0; 1/M_k; 0; -1/(M_k*l_k)];
end