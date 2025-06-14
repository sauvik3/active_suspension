function t = str_num2strn(x, n)
%STR_NUM2STRN Convert number to string of given length.
%   T = STR_NUM2STRN(X, N) converts the value X into a string representation T
%   with N characters using fixed point or scientific notation for maximal
%   achievable accuracy.
%
%   Examples:
%   str_num2strn( 2e-004/3, 9) = '66667e-09'
%   str_num2strn(-2e-004/3, 9) = '-66667e-9'
%   str_num2strn( 2e-003/3, 9) = '666667e-9'
%   str_num2strn(-2e-003/3, 9) = '-66667e-8'
%   str_num2strn( 2e-002/3, 9) = '.00666667'
%   str_num2strn(-2e-002/3, 9) = '-.0066667'
%   ...
%   str_num2strn(      2/3, 9) = '.66666667'
%   str_num2strn(     -2/3, 9) = '-.6666667'
%   str_num2strn(     20/3, 9) = '6.6666667'
%   str_num2strn(    -20/3, 9) = '-6.666667'
%   ...
%   str_num2strn( 2e+008/3, 9)  = ' 66666667'
%   str_num2strn(-2e+008/3, 9)  = '-66666667'
%   str_num2strn( 2e+009/3, 9)  = '666666667'
%   str_num2strn(-2e+009/3, 9)  = '-666667e3'
%   str_num2strn( 2e+010/3, 9)  = '6666667e3'
%   str_num2strn(-2e+010/3, 9)  = '-666667e4'
%
%   See also NUM2STR.

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               lib
% file name:             str_num2strn.m 
% author:                ASAM e.V.
%
%
% C by ASAM e.V., 2020
% Any use is limited to the scope described in the license terms.
% The license terms can be viewed at www.asam.net/license
%
% More Information on ASAM OpenCRG can be found here:
% https://www.asam.net/standards/detail/opencrg/
%
% *****************************************************************

if isnan(x)
    t(1:n) = '*';
    return
end

if x == 0
    t(1:n) = '0';
    return
end

xx = x;     % save original x value (for error messages/debugging)

while 1
    expo = floor(log10(abs(xx))) + 1;   % get exponent
    minus = xx < 0;                     % true if x negative
    nmant = n - minus;                  % sign eats one mantissa digit

    if expo < -2            % exponential notation with neg. exponent
        nmant = nmant - 3;      % neg. exponent eats three mantissa digits
        expo = expo - nmant;
        nexp = 1;
        if (expo <= -10)
            nmant = nmant - 1;  % second exponent digit eats further mantissa digit
            expo = expo + 1;
            nexp = 2;
        end
        if (expo <= -100)
            nmant = nmant - 1;  % third exponent digit eats further mantissa digit
            expo = expo + 1;
            nexp = 3;
        end
        mant = round(xx / 10^expo);
        t = sprintf('%*se%.*d', nmant+minus, int2str(mant), nexp, expo);
    elseif expo < 1         % fixed point notation with leading .
        nmant = nmant - 1;      % digital point eats one mantissa digit
        t = strrep(sprintf('%*.*f', n, nmant, xx), '0.', '.');
    elseif expo < nmant+1   % fixed point notation with leading digit(s)
        nmant = nmant - 1;      % digital point eats one mantissa digit
        t = sprintf('%*.*f', n, max(0, nmant-expo), xx);
    else                    % exponential notation with pos. exponent
        nmant = nmant - 2;      % pos. exponent eats two mantissa digits
        expo = expo - nmant;
        nexp = 1;
        if (expo >= 10)
            nmant = nmant - 1;  % second exponent digit eats further mantissa digit
            expo = expo + 1;
            nexp = 2;
        end
        if (expo >= 100)
            nmant = nmant - 1;  % third exponent digit eats further mantissa digit
            expo = expo + 1;
            nexp = 3;
        end
        mant = round(xx / 10^expo);
        t = sprintf('%*se%.*d', nmant+minus, int2str(mant), nexp, expo);
    end

    if nmant < 1            % no mantissa length remaining
        error('cannot convert %.17g to %d character field', x, n)
    end

    if length(t) == n       % heureka - it worked
        break;
    end

    if length(t) < n        % we should never end here
        error('internal error during convert %.17g to %d chararcter field', x, n)
    end

    % we achieved an extra leading 1 digit due to rounding, so loop again with
    % the rounded result achieved so far

    xx = str2double(t);
end
