/**
 * Copyright (c) 2025 Physical AI & Humanoid Robotics Textbook
 * Licensed under MIT License
 */

import React from 'react';

// LiteYoutube component for efficient YouTube embedding
const LiteYoutube = ({ videoId, title }) => {
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
};

export default LiteYoutube;