function fig_config(varargin)
% fig_config  Configure default figure and axis properties.
%
% =============================================================== %
% [INPUT] varargin: List of property-value pairs for customization.
%                   Note that it is case sensitive
%
% [PROPERTIES]                                                  [DEFAULT]
%    fontSize     : Default size of figure fonts.                    10
%    lineWidth    : Default line width.                              5
%    markerSize   : Default marker size.                             20
%    axesLineWidth: Default axis line width.                         1.8
% =============================================================== %
%
% =============================================================== %
% [OUTPUT] None
%
% =============================================================== %
%
% [EXAMPLES] 
%   (1) fig_config( 'fontSize', 12)
%   (2) fig_config( 'fontSize', 12, 'markerSize', 15)
%
% =============================================================== %
% SEE ALSO: testHelpFunction
%
% =============================================================== %
% CREATED BY: Moses C. Nah
% DATE      : 07-June-2020
% MODIFIED  : 29-Nov-2024
% =============================================================== %

% Input parser for custom settings
p = inputParser;

% Normalize varargin keys to lowercase
args = varargin;
for i = 1:2:length(args)
    if ischar(args{i}) || isstring(args{i})
        args{i} = lower(args{i});
    else
        error('Property names must be strings.');
    end
end
   
% Input validation for positive numeric values
validatePosNum = @(x) validateattributes( x, { 'numeric' }, { 'positive', 'scalar' } );

% Add parameters with default values
addParameter( p,      'fontsize',  10, validatePosNum );
addParameter( p,     'linewidth',   5, validatePosNum );
addParameter( p,    'markersize',  20, validatePosNum );
addParameter( p, 'axeslinewidth', 1.8, validatePosNum );

% Parse inputs
parse( p, varargin{ : } );
r = p.Results;

% Set default properties (global scope warning!)
set(0, 'defaultTextFontSize'            , 1.6 * r.fontsize );
set(0, 'defaultTextInterpreter'         , 'latex'          );
set(0, 'defaultLegendInterpreter'       , 'latex'          );
set(0, 'defaultLineLineWidth'           , r.linewidth      );
set(0, 'defaultLineMarkerSize'          , r.markersize     );
set(0, 'defaultAxesFontSize'            , r.fontsize       );
set(0, 'defaultAxesLineWidth'           , r.axeslinewidth  );
set(0, 'defaultAxesTickLabelInterpreter', 'latex'          );
set(0, 'defaultAxesXGrid'               , 'on'             );
set(0, 'defaultAxesYGrid'               , 'on'             );
set(0, 'defaultAxesZGrid'               , 'on'             );
set(0, 'defaultAxesBox'                 , 'on'             );
set(0, 'defaultFigureWindowStyle'       , 'normal'         );
set(0, 'defaultFigureUnits'             , 'normalized'     );
set(0, 'defaultFigurePosition'          , [0 0 1 1]        );
set(0, 'defaultFigureColor'             , [1 1 1]          );

% Add toolbar buttons and hide axis toolbar by default
set(0, 'defaultFigureCreateFcn', @(fig, ~) addToolbarExplorationButtons(fig));
set(0, 'defaultAxesCreateFcn', @(ax, ~) set(ax.Toolbar, 'Visible', 'off'));

end