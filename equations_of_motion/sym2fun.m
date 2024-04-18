%SYM2FUN Converts symbolic expressions to MATLAB anonymous functions.
%
% SYNTAX
%   f = sym2fun(fsym, vars, symVec1, repVec1, ..., symVecN, repVecN)
%   [f, fStr] = sym2fun(___)
%
% DESCRIPTION
%   sym2fun(fsym, vars, ...) converts the symbolic matrix fsym into an
%   anonymous function handle f. The function maps symbolic variables in 
%   fsym to specified replacements for numerical computation. Each pair
%   symVec and repVec contains symbolic variables and their corresponding
%   string replacements used in the anonymous function.
%
% INPUTS
%   fsym: NxM symbolic matrix.
%   vars: Cell array of strings, naming the input arguments for the
%         anonymous function.
%   symVec1, ..., symVecN: Vectors of symbolic variables.
%   repVec1, ..., repVecN: Cell arrays of strings for replacements.
%
% OUTPUTS
%   f: Anonymous function handle.
%   fStr: String representation of the function.
%
% EXAMPLE
%   x = sym('x%d', [2,1], 'real');
%   y = sym('y%d', [2,1], 'real');
%   p = [2*cos(x(1)) + 7*y(1)^2; sin(x(2)+y(2))];
%   J = jacobian(p, [x; y]) + randi([-10 10], 2, 4);
%   [f, fStr] = sym2fun(J,{'x','y'},x,{'x(1)','x(2)'},y,{'y(1)','y(2)'});
%
% COPYRIGHT AND DISCLAIMER
% This software (sym2fun.m) is provided "as is" and with all faults. The
% provider makes no representations or warranties of any kind concerning
% the safety, suitability, lack of viruses, inaccuracies, typographical
% errors, or other harmful components of this software. There are inherent
% dangers in the use of any software, and you are solely responsible for
% determining whether this software product is compatible with your
% equipment. You are also responsible for the protection of your equipment
% and backup of your data. The provider will not be liable for any damages
% you may suffer in connection with using, modifying, or distributing this
% software product.
%
% All information provided is on an "as is" basis. The author (Siamak G. Faal)
% makes no representations as to the accuracy or completeness of any
% information provided. The author will not be liable for any errors, omissions,
% or delays in this information or any losses, injuries, or damages arising from
% its use. Your use of any information or examples is entirely at your own risk.
% Should the software prove defective, you assume the cost of all necessary
% servicing, repair, or correction.


function [f, fStr] = sym2fun(fsym, vars, varargin)

    % Validate the inputs
    assert(mod(length(varargin), 2) == 0, ...
        'Replacement arguments must be in pairs.');

    [N, M] = size(fsym); % Extract the size of the symbolic matrix
    fStrArray = strings(N, M); % Preallocate the output expression

    % Process each element in the symbolic matrix
    for i = 1:N
        for j = 1:M
            % Convert the symbolic expression to a string
            str = char(fsym(i, j));
            
            % Apply all replacements
            for k = 1:2:length(varargin)
                expression = varargin{k};
                substitute = varargin{k + 1};
                for r = 1:length(expression)
                    str = regexprep(str, char(expression(r)), substitute{r});
                end
            end
            
            % Store the modified string
            fStrArray(i, j) = string(str);
        end
    end

    % Join all expressions into a single string representing the function
    rowStrs = join(fStrArray, ',');
    fullStr = join(rowStrs, ';');
    fStr = sprintf('[%s]', fullStr);

    % Create input string for the anonymous function
    inputs = strjoin(vars, ',');

    % Create the function handle
    f = str2func(sprintf('@(%s) %s', inputs, fStr));
end