/**
 * Copyright (c) 2017-present, Facebook, Inc.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

const React = require('react');

class Footer extends React.Component {
  docUrl(doc, language) {
    const baseUrl = this.props.config.baseUrl;
    return baseUrl + 'docs/' + (language ? language + '/' : '') + doc;
  }

  pageUrl(doc, language) {
    const baseUrl = this.props.config.baseUrl;
    return baseUrl + (language ? language + '/' : '') + doc;
  }

  render() {
    const currentYear = new Date().getFullYear();
    return (
      <footer className="nav-footer" id="footer">
        <section className="sitemap">
          <a href={this.props.config.baseUrl} className="nav-home">
            {this.props.config.footerIcon && (
              <img
                src={this.props.config.baseUrl + this.props.config.footerIcon}
                alt={this.props.config.title}
                width="66"
                height="58"
              />
            )}
          </a>
         <div>
            <h5>Docs</h5>
            <a href="https://ihmcroboticsdocs.github.io/ihmc-open-robotics-software/docs/quickstarthome.html">
              Quick Start
            </a>
            <a href="https://ihmcroboticsdocs.github.io/docs/docshome.html">
              Software Documentation
            </a>
          </div>
          <div>
            <h5>Community</h5>
            <a href="https://github.com/ihmcrobotics">
              GitHub
            </a>
            <a
              href="https://www.facebook.com/TheIHMC"
              target="_blank"
              rel="noreferrer noopener">
              Facebook
            </a>
            <a
              href="https://twitter.com/ihmcrobotics"
              target="_blank"
              rel="noreferrer noopener">
              Twitter
            </a>
             <a
              href="https://www.youtube.com/user/DRCihmcRobotics"
              target="_blank"
              rel="noreferrer noopener">
              YouTube
            </a>
          </div>
        </section>
        <section className="copyright">{this.props.config.copyright}</section>
      </footer>
    );
  }
}

module.exports = Footer;
