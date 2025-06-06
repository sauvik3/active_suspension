%% ----------------- Get State-Space Model -----------------
function [A, B] = get_state_space(m_s, m_u, k_s, k_t, b_s)
    A = [ 0 1 0 0;
         -k_s/m_s -b_s/m_s k_s/m_s b_s/m_s;
          0 0 0 1;
          k_s/m_u b_s/m_u -(k_s + k_t)/m_u -b_s/m_u];

    B = [0; 1/m_s; 0; -1/m_u];

    % Small regularization to avoid singularities
    A = A + 1e-8 * eye(size(A));
    B = B + 1e-8;
end
