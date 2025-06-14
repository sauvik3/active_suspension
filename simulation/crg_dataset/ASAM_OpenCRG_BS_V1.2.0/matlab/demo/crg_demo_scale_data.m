function [data] = crg_demo_scale_data(data)
% CRG_DEMO_SCALE_DATA scale surface data
%   [DATA] = CRG_SCALE_DATA(DATA) checks CRG data
%   and perform a scaling to the data.
%
%   Inputs:
%   DATA     struct array as defined in CRG_INTRO.
%
%   Outputs:
%   DATA    scaled dat
%
%   Examples:
%   data = crg_demo_scale_data(data);
%
%   See also CRG_PERFORM2SURFACE, CRG_CHECK_UV_DESCRIPT,
%            CRG_INTRO.

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               demo
% file name:             crg_demo_scale_data.m
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

%% check number of arguments

error(nargchk(1,1,nargin));

%% check if already succesfully checked

if ~isfield(data, 'ok')
    data = crg_check(data);
    if ~isfield(data, 'ok')
        error('CRG:checkError', 'check of DATA was not completely successful')
    end
end

%% build uv

ubeg = data.head.ubeg;
uend = data.head.uend;
uinc = data.head.uinc;
u = ubeg:uinc:uend;

if isfield(data.head, 'vinc')
    vmin = data.head.vmin;
    vmax = data.head.vmax;
    vinc = data.head.vinc;
    v = vmin:vinc:vmax;
else
    v = data.v;
end

vmin = v(1);
vmax = v(end);
vn   = size(v,2);

%% create longitudenal and lateral profile(s) like this and do the scaling

% u ----> coordinate       [  begin                                 middle sections                       end   ]
% v ----> coordinate       [  left                                  origin                                right ]
%  l ---> left  hand side
%  r ---> right hand side
%  w ---> width of whole road
%  c ---> center of road
%   p --> profile
%    _ -> name_sect
%    _ -> name_prof
%                 offset or ampltitude to origin
%                 |
uwp_scal_sect  =          [  ubeg                ubeg+(uend-ubeg)*[0.3  1/3  2/3  0.7]                    uend  ];    % road width       u sections
uwp_scal_prof  =  1     * [  1                                     1    2    2    1                       1     ];
vwp_scal_sect  =          [  fliplr(v)                                                                          ];    % road width       v sect
vwp_scal_prof  =  1     * [  ones(size(vwp_scal_sect))                                                          ];


uwp_edge_sect  =          [  ubeg   ubeg+(uend-ubeg)*0.05                         uend-(uend-ubeg)*0.05   uend  ];    % road width       u sections
uwp_edge_prof  =  1     * [  0                       1                                             1      0     ];
vwp_edge_sect  =          [  fliplr(v)                                                                          ];    % road width       v sect
vwp_edge_prof  =  1     * [  min([linspace(0,0.1*(vn-1),vn); ones(1,vn); linspace(0.1*(vn-1),0,vn)],[],1)       ];

%% combine all longitudenal and lateral profile(s) like this

%              mode one of {'Profile' 'Random' 'Ignore' }
%              |          u section       u profile         v section       v profile       valid only for random profile(s)
%              |          |               |                 |               |               /----------^----------\
uv_surf = { ...
          ; { 'Profile' [ uwp_scal_sect ; uwp_scal_prof ] [ vwp_scal_sect ; vwp_scal_prof ]                         } ...
          ; { 'Profile' [ uwp_edge_sect ; uwp_edge_prof ] [ vwp_edge_sect ; vwp_edge_prof ]                         } ...
          };

%% check only the uv description

crg_check_uv_descript(uv_surf, {'Ignore' 'Profile' 'Random'});

%% perform the scaling

data = crg_perform2surface(data, uv_surf, 'mult');
txtnum = length(data.ct) + 1; data.ct{txtnum} = 'CRG perform scaling to surface';

%% check and write data to file

txtnum = length(data.ct) + 1; data.ct{txtnum} = 'scaling finished';
data = crg_single(data);
data = crg_check(data);

end
