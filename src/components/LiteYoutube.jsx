/**
 * Copyright (c) 2025 Physical AI & Humanoid Robotics Textbook
 * Licensed under MIT License
 */

import React, { useState } from 'react';
import { useColorMode } from '@docusaurus/theme-common';

// LiteYoutube component for efficient YouTube embedding
const LiteYoutube = ({ videoId, title }) => {
  const { colorMode } = useColorMode();
  const [loaded, setLoaded] = useState(false);

  const loadVideo = () => {
    setLoaded(true);
  };

  if (loaded) {
    return (
      <div className="video-container">
        <iframe
          width="560"
          height="315"
          src={`https://www.youtube.com/embed/${videoId}`}
          title={title}
          frameBorder="0"
          allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
          allowFullScreen
        ></iframe>
      </div>
    );
  }

  const thumbnailStyle = {
    backgroundImage: `url(https://i.ytimg.com/vi/${videoId}/hqdefault.jpg)`
  };

  return (
    <button className="lite-youtube" style={{...thumbnailStyle, position: "relative"}} onClick={loadVideo}>
      <div className="lty-playbtn"></div>
      <small>{title}</small>
    </button>
  );
};

export default LiteYoutube;