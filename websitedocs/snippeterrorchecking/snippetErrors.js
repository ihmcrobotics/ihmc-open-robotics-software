

//For this to work, npm install isomorphic-fetch, jsdom, and jquery

fetch = require('isomorphic-fetch');
var jsdom = require('jsdom');
const { JSDOM } = jsdom;
const { window } = new JSDOM();
const { document } = (new JSDOM('')).window;
global.document = document;
var $ = jQuery = require('jquery')(window);
var matchIndex, allCodeBlocks;

function checkSnippet() {
	var script = document.getElementById("snippetscript");
	var sources = eval(script.getAttribute('sources'));
	var numberOfSources = sources.length;

	// Getting the attributes from all the code blocks
	allCodeBlocks = Array.from(document.getElementsByTagName('code'));
	var urls = []; // array of data from each source

	// Throws error on invalid url
	function notValidURL(response) {
		if (!response.ok) {
			process.exitCode = 1;
			throw Error(response.statusText);
		}
		return response;
	}

	// Get and operate on data from source files
	for (var i = 0; i < numberOfSources; i++) {
		urls[i] = fetch(sources[i]).then(notValidURL).then(function(response) {
			return response.text()
		});
	}

	Promise
			.all(urls)
			.then(
					function(values) {
						for (var urlIndex = 0; urlIndex < numberOfSources; urlIndex++) {
							! function(urlIndex) {
							var dataFromSource = values[urlIndex];
							matchIndex = allCodeBlocks.findIndex(function(
									element) {
								return element.getAttribute('data-url-index') == urlIndex
										.toString();
							});

							// For each URL, find all the code blocks with matching
							// data-url-index
							while (matchIndex != -1) {

								// Change inner HTML of code block
								var codeBlock = allCodeBlocks[matchIndex];
								var typeOfSnippet = codeBlock
										.getAttribute('data-snippet');
								var startIndex, endindex;

								 if (typeOfSnippet == "portion") {
									startIndex = dataFromSource.indexOf(codeBlock
											.getAttribute('data-start'));
									if(startIndex < 0) {
										console.log("Start string not found at element id: " + codeBlock.id);
										process.exitCode = 1;
                    allCodeBlocks.splice(matchIndex, 1);
                    matchIndex = allCodeBlocks
                      .findIndex(function(element) {
                        return element
                          .getAttribute('data-url-index') == urlIndex
                          .toString();
                      });
                    continue;
									}

									// Substring from index to the rest of file
									if (codeBlock.getAttribute('data-end') != null) {
										endIndex = dataFromSource.indexOf(codeBlock
												.getAttribute('data-end'),
												startIndex);
										if(endIndex < 0) {
											console.log("End string not found at element id: " + codeBlock.id);
											process.exitCode = 1;
                      allCodeBlocks.splice(matchIndex, 1);
                      matchIndex = allCodeBlocks
                        .findIndex(function(element) {
                          return element
                            .getAttribute('data-url-index') == urlIndex
                            .toString();
                        });
                      continue;
									}
								}
								} else if (typeOfSnippet == "multipleportions")
								{
									var portions = eval(codeBlock
											.getAttribute('data-portions'));
									for (j = 0; j < portions.length; j++) {
										startIndex = dataFromSource
												.indexOf(portions[j][0]);
										if(startIndex < 0) {
											console.log("Start string not found at element id: " + codeBlock.id);
											process.exitCode = 1;
                      continue;
										}
										// Substring with start index to rest of
										// file
										if (portions[j].length > 1) {
											endIndex = dataFromSource.indexOf(
													portions[j][1], startIndex);
											if(endIndex < 0) {
												console.log("End string not found at element id: " + codeBlock.id);
												process.exitCode = 1;
                        continue;
											}
										}
									}
								}
								allCodeBlocks.splice(matchIndex, 1);

								matchIndex = allCodeBlocks  
										.findIndex(function(element) {
											return element
													.getAttribute('data-url-index') == urlIndex
													.toString();
										});
							}
					}(urlIndex);
				}
			});
}
		//using shell script
		var webpageUrl = process.argv[2];
		$.get(webpageUrl, function(data) {
			rawHTML = $.parseHTML(data, document, true);
			document.body.appendChild(rawHTML[rawHTML.length-1]);
			checkSnippet();
		});
