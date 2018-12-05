hljs.initHighlightingOnLoad();

var script = document.getElementById("snippetscript");
var sources = eval(script.getAttribute('sources'));
var numberOfSources = sources.length;

// Getting the attributes from all the code blocks
var allCodeBlocks = Array.from(document.getElementsByTagName('code'));
var urls = []; // array of data from each source

// Throws error on invalid url
function notValidURL(response) {
	if (!response.ok) {
		throw Error(response.statusText);
	}
	return response;
}

// Get and operate on data from source files
for (i = 0; i < numberOfSources; i++) {
	urls[i] = fetch(sources[i]).then(notValidURL).then(function(response) {
		return response.text()
	});
}

Promise
		.all(urls)
		.then(
				function(values) {
					for (i = 0; i < numberOfSources; i++) {
						var dataFromSource = values[i];
						var matchIndex = allCodeBlocks.findIndex(function(
								element) {
							return element.getAttribute('data-url-index') == i
									.toString();
						});

						// For each URL, find all the code blocks with matching
						// data-url-index
						while (matchIndex != -1) {

							// Change inner HTML of code block
							var codeBlock = allCodeBlocks[matchIndex];
							var typeOfSnippet = codeBlock
									.getAttribute('data-snippet');
							var codeChunk = "";
							var startIndex, endindex;

							if (typeOfSnippet == "complete") {
								codeChunk = dataFromSource;
							} else if (typeOfSnippet == "portion") {
								startIndex = dataFromSource.indexOf(codeBlock
										.getAttribute('data-start'));
								if(startIndex < 0) throw "Start string not found at element id: " + codeBlock.id;

								// Substring from index to the rest of file
								if (codeBlock.getAttribute('data-end') === null) {
									codeChunk = dataFromSource
											.substring(startIndex);
								} else {
									endIndex = dataFromSource.indexOf(codeBlock
											.getAttribute('data-end'),
											startIndex);
									if(endIndex < 0) throw "End string not found at element id: " + codeBlock.id;
									codeChunk = dataFromSource.substring(
											startIndex, endIndex);
								}
							} else
							// If the snippet involves multiple portions,
							// data-snippet="multipleportions"
							{
								var portions = eval(codeBlock
										.getAttribute('data-portions'));
								for (j = 0; j < portions.length; j++) {
									startIndex = dataFromSource
											.indexOf(portions[j][0]);
									if(startIndex < 0) throw "Start string not found at element id: " + codeBlock.id;

									// Substring with start index to rest of
									// file
									if (portions[j].length == 1) {
										codeChunk = codeChunk
												+ "\n"
												+ dataFromSource
														.substring(startIndex);
									} else {
										endIndex = dataFromSource.indexOf(
												portions[j][1], startIndex);
										if(endIndex < 0) throw "End string not found at element id: " + codeBlock.id;
										codeChunk = codeChunk
												+ "\n"
												+ dataFromSource.substring(
														startIndex, endIndex);
									}
								}
							}
							codeBlock.innerHTML = hljs.highlight('java',
									codeChunk).value;
							allCodeBlocks.splice(matchIndex, 1);

							matchIndex = allCodeBlocks
									.findIndex(function(element) {
										return element
												.getAttribute('data-url-index') == i
												.toString();
									});
						}
					}
				});
