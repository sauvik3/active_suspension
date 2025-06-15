function [data] = crg_wrap(data)
%CRG_WRAP Wrap heading angles to +/- pi range.
%   [DATA] = CRG_WRAP(DATA) wraps OpenCRG heading angle data in road parameters
%   and road data to a +/- pi range.
%
%   Inputs:
%   DATA    struct array as defined in CRG_INTRO.
%
%   Outputs:
%   DATA    struct array with updated heading angles.
%
%   Examples:
%   data = crg_wrap(data);
%       Wraps heading angles in road parameters and road data to a +/- pi range.
%
%   See also CRG_INTRO.

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               lib
% file name:             crg_wrap.m 
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

%% convert core vectors and arrays to single type

% head data
if isfield(data.head, 'pbeg')
    data.head.pbeg = atan2(sin(data.head.pbeg), cos(data.head.pbeg));
end
if isfield(data.head, 'pend')
    data.head.pend = atan2(sin(data.head.pend), cos(data.head.pend));
end

% core reference line data
if isfield(data, 'p')
    if isa(data.p, 'double')
        data.p = atan2(sin(data.p), cos(data.p));
    else
        data.p = single(atan2(sin(double(data.p)), cos(double(data.p))));
    end
end

end
