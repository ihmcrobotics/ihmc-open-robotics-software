function  selftest_yamlmatlab(varargin)
% This function tests consistency of YAMLMatlab, by default, the results
% are stored in selftest_report.html in current work folder.
% Example
% >> selftest_yamlmatlab()
% >> selftest_yamlmatlab(outFileName)
%
%  %======================================================================
%{
		Copyright (c) 2011
		This program is a result of a joined cooperation of Energocentrum
		PLUS, s.r.o. and Czech Technical University (CTU) in Prague.
        The program is maintained by Energocentrum PLUS, s.r.o. and
        licensed under the terms of MIT license. Full text of the license
        is included in the program release.
		
        Author(s):
		Jiri Cigler, Dept. of Control Engineering, CTU Prague 
		Jan  Siroky, Energocentrum PLUS s.r.o.
		
        Implementation and Revisions:

        Auth  Date        Description of change
        ----  ---------   -------------------------------------------------
        jc    25-May-11   First implementation        
%}
%======================================================================

    fprintf('Running tests.\n');
    outFname = 'selftest_report.html';
    if numel(varargin)
        outFname = varargin{1};
    end

    outStr = getHTMLHeader();

    outStr = strcat(outStr,'<h1>Selftest report from:',datestr(now),'</h1>');

    tests = dir([fileparts(which('selftest_yamlmatlab')) filesep 'test*.m']);

    for test=tests'
        [~,func]=fileparts(test.name);

        fhandle = str2func(func);
        stat = fhandle();
    
        outStr = strcat(outStr, '<div id="MainTest"> <h2>',func, '</h2>', stat2html(stat,func),'</div>');
    
    end

    outStr = strcat(outStr,'</BODY></HTML>');


    fid = fopen(outFname,'w');
    fprintf(fid,outStr);
    fclose(fid);
    
    fprintf('Opening internal browser.\n');
   
    web(outFname,'-new');
end

function html = stat2html(stat,name)
    if not(isstruct(stat))
        error('Input argument must be a struct');
    end

    html = '';

    fn = fieldnames(stat);
    if all(ismember({'ok','desc'},fn))
        if stat.ok
            flag = 'Passed';
        else
            flag = '<b style="color:red">Failed</b>,';
        end
        html = strcat(html,'<div id="Test"><h3>',name,': </h3> ', flag, ' <i>', stat.desc,'</i> </div>' );
    end
    html = [html, '<table>'];
    for test = setdiff(fn',{'ok','desc'})
        html = [html, '<tr>'];
        html = strcat(html, stat2html(stat.(test{1}),test{1}));
        html = [html, '</tr>'];
    end
    html = [html, '</table>'];
end


function str = getHTMLHeader()
    str = [ '<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01//EN"   "http://www.w3.org/TR/html4/strict.dtd">' ... 
        '<HTML> ' ... 
        '<HEAD>'... 
        '<TITLE>::SELFTEST REPORT::</TITLE><STYLE> H2{color:blue} #MainTest{border: 1px blue solid;} h3,h4,h5,h6 {display: inline;}     </STYLE>'... 
        '</HEAD><BODY style="font-family:Arial, helvetica">'];
end