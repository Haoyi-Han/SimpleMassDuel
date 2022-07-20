function pointlist1 = importfile1(filename, dataLines)
%IMPORTFILE1 从文本文件中导入数据
%  POINTLIST1 = IMPORTFILE1(FILENAME)读取文本文件 FILENAME 中默认选定范围的数据。
%  以表形式返回数据。
%
%  POINTLIST1 = IMPORTFILE1(FILE, DATALINES)按指定行间隔读取文本文件 FILENAME
%  中的数据。对于不连续的行间隔，请将 DATALINES 指定为正整数标量或 N×2 正整数标量数组。
%
%  示例:
%  pointlist1 = importfile1("E:\Lab\SimpleMassDuel\utils\visualizer-matlab\point_list.dat", [1, Inf]);
%
%  另请参阅 READTABLE。
%
% 由 MATLAB 于 2022-06-15 17:11:54 自动生成

%% 输入处理

% 如果不指定 dataLines，请定义默认范围
if nargin < 2
    dataLines = [1, Inf];
end

%% 设置导入选项并导入数据
opts = delimitedTextImportOptions("NumVariables", 4);

% 指定范围和分隔符
opts.DataLines = dataLines;
opts.Delimiter = " ";

% 指定列名称和类型
opts.VariableNames = ["x", "y", "r", "color"];
opts.VariableTypes = ["double", "double", "double", "double"];

% 指定文件级属性
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
opts.ConsecutiveDelimitersRule = "join";
opts.LeadingDelimitersRule = "ignore";

% 导入数据
pointlist1 = readtable(filename, opts);

end