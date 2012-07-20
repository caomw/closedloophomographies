function varargout = loadMatFromOpenCVXML(filename, first, last)
    for k = first:last
        filepath = sprintf('%s%04d.xml', filename, k)
        varargout{k - first + 1} = importXMLtoMATLAB(filepath);
    end
end