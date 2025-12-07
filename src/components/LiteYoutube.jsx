/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

import React, {useState, useRef, useEffect} from 'react';

import './LiteYTEmbed.scss';

/*
 * CSS is copied from YouTube Embed's package lite-youtube-embed
 * License: https://github.com/paulirish/lite-youtube-embed/blob/master/LICENSE
 */
const LiteYTEmbedCSS = `
.lite-youtube {
  background-color: #000;
  position: relative;
  display: block;
  contain: content;
  background-position: center center;
  background-size: cover;
  cursor: pointer;
  max-width: 720px;
}

.lite-youtube::before {
  content: '';
  display: block;
  position: absolute;
  top: 0;
  background-image: url(data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAADGCAYAAAARSr8IAAAAVklEQVR4nGNgGAUgCsx8CII/yJj///9nEO6Bsv///2dA5Py//+FPxQPpf///n0G4H0SASAKp/yD+AykIJhJrBpQG00A8C8QCEbu1AI9FAJtpvC9xaAAAAABJRU5ErkJggg==);
  background-repeat: no-repeat;
  background-size: 68px 48px;
  z-index: 1;
  width: 68px;
  height: 48px;
  margin: auto;
  inset: 0;
  transition: .4s cubic-bezier(0,0,0.2,1);
}

.lite-youtube:hover::before {
  background-image: url(data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmci viZXdCb3g9IjAgMCA2NCA0OCI+PHBhdGggZD0iTTIyIDMuNGMxLjctLjEgMy40LS4xIDUuMi0uMXMyLjcuMSA0LjEuMUwyOCAxMi4ydjIzLjZMNTAuNiAyNC4xbC0yLjktMi45Yy0uOC0uOC0xLjgtMS4yLTIuOS0xLjJIMjh6IiBmaWxsPSIjZmZmIi8+PC9zdmc+);
}

.lite-youtube::after {
  content: "";
  display: block;
  padding-bottom: calc(100% / (16 / 9));
}

.lite-youtube > iframe {
  width: 100%;
  height: 100%;
  position: absolute;
  top: 0;
  left: 0;
  border: 0;
}

.lytbtn {
  display: block;
  width: 68px;
  height: 48px;
  position: absolute;
  cursor: pointer;
  z-index: 0;
  opacity: 0.8;
  transition: all 0.4s cubic-bezier(0,0,0.2,1);
  border: 0;
}

.lytbtn:hover {
  opacity: 1;
  transform: scale(1.06);
}

.lytbtn::after {
  content: "";
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate3d(-50%, -50%, 0);
  width: 46%;
  height: 32%;
  background: #ff0000;
  clip-path: polygon(0% 0%, 100% 50%, 0% 100%);
}
`;

const LiteYoutube = ({videoId, title = 'YouTube video'}) => {
  const [iframeLoaded, setIframeLoaded] = useState(false);
  const [cssLoaded, setCssLoaded] = useState(false);
  const ref = useRef(null);

  useEffect(() => {
    if (iframeLoaded) {
      return;
    }

    if (!ref.current) {
      return;
    }

    const embedDiv = ref.current.querySelector('.lite-youtube');
    const iframe = document.createElement('iframe');
    iframe.className = 'yt-iframe';
    iframe.width = '560';
    iframe.height = '315';
    iframe.loading = 'lazy';
    iframe.src = `https://www.youtube.com/embed/${videoId}?autoplay=1&rel=0&showinfo=0`;
    iframe.title = title;
    iframe.allow =
      'accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture';
    iframe.allowFullscreen = true;

    // When iframe is loaded, replace the embed div with the iframe
    iframe.onload = () => {
      if (ref.current) {
        ref.current.replaceChild(iframe, embedDiv);
        setIframeLoaded(true);
      }
    };
  }, [iframeLoaded, videoId, title]);

  useEffect(() => {
    // Dynamically inject the LiteYouTubeEmbedCSS
    if (cssLoaded) {
      return;
    }

    const style = document.createElement('style');
    style.innerHTML = LiteYTEmbedCSS;
    document.head.appendChild(style);

    setCssLoaded(true);

    // Cleanup the style element when component unmounts
    return () => {
      document.head.removeChild(style);
    };
  }, [cssLoaded]);

  return (
    <div ref={ref}>
      <div
        className="lite-youtube"
        style={{
          backgroundImage: `url(https://i.ytimg.com/vi/${videoId}/hqdefault.jpg)`,
        }}>
        <button type="button" className="lytbtn" />
      </div>
    </div>
  );
};

export default LiteYoutube;