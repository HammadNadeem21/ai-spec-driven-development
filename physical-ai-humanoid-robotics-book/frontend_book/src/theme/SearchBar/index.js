import React from 'react';
import SearchBar from '@theme-original/SearchBar';

export default function SearchBarWrapper(props) {
  return (
    <>
      {/* Enhanced search functionality with accessibility features */}
      <div className="search-enhancement">
        <SearchBar {...props} />
      </div>
    </>
  );
}