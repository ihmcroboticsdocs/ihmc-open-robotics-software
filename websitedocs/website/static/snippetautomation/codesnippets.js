hljs.initHighlightingOnLoad();

//Get the array of urls
var allScripts = document.getElementsByTagName('script');
var script = allScripts[allScripts.length-1];
var sources = eval(script.getAttribute('sources'));
var numberOfSources = sources.length;

//Getting the attributes from all the code blocks
var allCodeBlocks = Array.from(document.getElementsByTagName('code'));
var urls = []; //array of data from each source

//Get and operate on data from source files
for(i = 0; i < numberOfSources; i++) 
	{
	urls[i] = fetch(sources[i]).then(function(response) {return response.text()});
	}
Promise.all(urls).then(function(values) {
	for(i = 0; i < numberOfSources; i++)
		{
		var dataFromSource = values[i];
		var matchIndex = allCodeBlocks.findIndex(function(element) {
			return element.getAttribute('data-url-index') == i.toString();
			});	
		
		//For each URL, find all the code blocks with matching data-url-index 
		while(matchIndex != -1) {				
			//Change inner HTML of code block
			var codeBlock = allCodeBlocks[matchIndex];
			if(codeBlock.getAttribute('data-snippet') == "complete")
				{
				codeBlock.innerHTML = hljs.highlight('java', dataFromSource).value;
				}
			else 
				{
				var startIndex = dataFromSource.indexOf(codeBlock.getAttribute('data-start'));
				var endIndex = dataFromSource.indexOf(codeBlock.getAttribute('data-end'), startIndex);
				var codeChunk = dataFromSource.substring(startIndex, endIndex);
				codeBlock.innerHTML = hljs.highlight('java', codeChunk).value;
				}
			allCodeBlocks.splice(matchIndex, 1);  //removes the element after operating on it
			
			matchIndex = allCodeBlocks.findIndex(function(element) {
				return element.getAttribute('data-url-index') == i.toString();
				});	
			}
		}
});

